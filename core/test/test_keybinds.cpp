#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "Keybinds.hpp"

#include <QGuiApplication>

TEST_CASE("G is documented as the last-growth overlay toggle")
{
    using namespace vc3d::keybinds;

    qputenv("QT_QPA_PLATFORM", "offscreen");
    int argc = 1;
    char applicationName[] = "test_keybinds";
    char* argv[] = {applicationName, nullptr};
    QGuiApplication application(argc, argv);

    CHECK(sequenceFor(keypress::ToggleLastGrowthOverlay) == QKeySequence(Qt::Key_G));
    CHECK(buildKeybindsHelpText().contains(
        QStringLiteral("G: Toggle last growth overlay (tap)")));
}
