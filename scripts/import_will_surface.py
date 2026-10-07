#!/usr/bin/env python3
"""Import a Will atlas native-xyz TIFF as an editable, native VC3D project.

Copies float32 XYZ without resampling. The source remains untouched. Requires
numpy and tifffile (the existing Vesuvius .venv contains both).
"""
import argparse
import io
import json
from pathlib import Path
from urllib.parse import parse_qs, quote, urlsplit
from urllib.request import urlopen

import numpy as np
import tifffile


def fetch(url):
    with urlopen(url, timeout=30) as response:
        return response.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--atlas-url', help='Full atlas URL including ?patch=...')
    source.add_argument('--xyz-file', type=Path, help='Downloaded native-xyz.tif')
    parser.add_argument('--volume-url', help='Native multiscale CT Zarr URL')
    parser.add_argument('--output', type=Path, required=True, help='New project directory')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists. Choose a new directory to preserve existing edits.')
    case = None
    if args.atlas_url:
        url = urlsplit(args.atlas_url)
        patch = parse_qs(url.query).get('patch', [None])[0]
        if url.scheme not in ('http', 'https') or not patch:
            parser.error('Use an http(s) atlas URL with a patch parameter.')
        base = f'{url.scheme}://{url.netloc}/api/layers/'
        catalog = json.loads(fetch(base + 'catalog.json'))
        case = next((c for c in catalog['cases'] if c['id'] == patch), None)
        if case is None:
            parser.error('The selected patch is absent from the atlas catalog.')
        relative = case['nativeCoordinates']
        if relative.startswith('/') or '..' in relative.split('/') or '://' in relative:
            parser.error('The catalog contains an invalid XYZ path.')
        raw = fetch(base + quote(relative, safe='/'))
    else:
        raw = args.xyz_file.read_bytes()
    with tifffile.TiffFile(io.BytesIO(raw)) as image:
        metadata = json.loads(image.pages[0].description)
        points = image.asarray()
    if points.dtype != np.float32 or points.ndim != 3 or points.shape[2] != 3:
        parser.error('Expected an H x W x 3 float32 native XYZ grid.')
    if metadata.get('coordinateFrame') != 'volume-native-voxels':
        parser.error('XYZ coordinates must be in the native CT voxel frame.')
    transform = np.asarray(metadata['imageToGrid'], dtype=float)
    if transform.shape != (2, 3) or not np.isfinite(transform).all() or \
       np.any(transform[:, 2] != 0) or transform[0, 1] != 0 or transform[1, 0] != 0 or \
       min(transform[0, 0], transform[1, 1]) <= 0:
        parser.error('This XYZ grid has a rotated or translated chart; import its original tifxyz instead.')
    if not np.isfinite(points).all():
        parser.error('XYZ grid contains nonfinite coordinates.')
    valid = (points >= 0).all(axis=2)
    if not valid.any() or np.any(~valid & (points != -1).any(axis=2)):
        parser.error('Expected valid native coordinates or the (-1,-1,-1) hole sentinel.')
    original_id = metadata['caseId']
    if Path(original_id).name != original_id or original_id in ('.','..'):
        parser.error('Invalid segment identifier.')
    segment_id = original_id + '-manual'
    volume_url = args.volume_url
    if not volume_url and case:
        volume_url = ('https://vesuvius-challenge-open-data.s3.amazonaws.com/' +
                      quote(case['scroll'],safe='') + '/volumes/' + quote(metadata['volumeId'],safe=''))
    if not volume_url:
        parser.error('--volume-url is required with --xyz-file.')
    if not volume_url.rstrip('/').endswith('.zarr'):
        parser.error('The volume URL must identify a multiscale .zarr volume.')
    root = args.output.resolve()
    segment = root / 'segments' / segment_id
    segment.mkdir(parents=True)
    for index, name in enumerate('xyz'):
        tifffile.imwrite(segment / f'{name}.tif', points[:, :, index], compression='deflate')
    meta = dict(format='tifxyz', type='seg', uuid=segment_id,
                scale=[float(transform[0,0]),float(transform[1,1])],
                bbox=[points[valid].min(0).tolist(),points[valid].max(0).tolist()],
                source_case_id=original_id, source_coordinate_frame='volume-native-voxels',
                source_volume=volume_url, voxelsize=metadata['voxelSizeUm'])
    (segment/'meta.json').write_text(json.dumps(meta,indent=2)+'\n')
    project = dict(name='Will surface · manual extension', version=1,
                   volumes=[dict(location=volume_url,tags=[f"vc-open-data-voxel-size-um:{metadata['voxelSizeUm']}"])],
                   segments=['segments'], output_segments='segments', normal_grids=[],
                   remote_cache_root=str(root/'ct-cache'))
    path = root/'project.volpkg.json'
    path.write_text(json.dumps(project,indent=2)+'\n')
    print(path)


if __name__ == '__main__':
    main()
