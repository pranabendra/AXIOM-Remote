#include "catch2/catch.hpp"

#include "PainterMod.h"

TEST_CASE("DrawFillRectangle test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawFillRectangle(0, 0, 10, 10, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawLine test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawLine(0, 0, 10, 10, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawFastVLine test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawFastVLine(0, 0, 10, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawFastHLine test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawFastHLine(0, 0, 10, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawFillRoundRectangle test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawFillRoundRectangle(0, 0, 10, 10, 3, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}


TEST_CASE("DrawRectangle test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawRectangle(0, 0, 10, 10, 0xFFFF);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawStripedRectangle test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawStripedRectangle(0, 0, 10, 10, 0x0, 0x0, 0, 0);

    REQUIRE(painter.wrongDirection == false);
}

// TEST_CASE("DrawCircle test")
// {
//     uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
//     PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

//     painter.DrawCircle(0, 0, 10, 0x0);

//     REQUIRE(painter.wrongDirection == false);
// }

// TEST_CASE("DrawFillCircle test")
// {
//     uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
//     PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

//     painter.DrawFillCircle(20, 20, 10, 0x0);

//     REQUIRE(painter.wrongDirection == false);
// }

// TEST_CASE("DrawCircleQuarter test")
// {
//     uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
//     PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

//     painter.DrawCircleQuarter(20, 20, 10, 1, 0x0);
//     painter.DrawCircleQuarter(20, 20, 10, 2, 0x0);
//     painter.DrawCircleQuarter(20, 20, 10, 4, 0x0);
//     painter.DrawCircleQuarter(20, 20, 10, 8, 0x0);

//     REQUIRE(painter.wrongDirection == false);
// }

// TEST_CASE("DrawText test")
// {
//     uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
//     PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

//     painter.DrawText(0, 0, "Test", 0x0, Font::FreeSans9pt7b, TextAlign::TEXT_ALIGN_LEFT, 10);

//     REQUIRE(painter.wrongDirection == false);
// }

TEST_CASE("Fill test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.Fill(0x0);

    REQUIRE(painter.wrongDirection == false);
}

TEST_CASE("DrawPixel test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);
    
    painter.DrawPixel(20,30, 0x53);

    REQUIRE(painter.GetPixel(20, 30) == 0x53);
}

// Testcase for DrawIcon() method
// Checks whether the icon is contained in the screen completely or if some part is missing from the screen
TEST_CASE("DrawIcon test")
{
    uint16_t* framebuffer = new uint16_t[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT];
    PainterMod painter(framebuffer, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT);

    painter.DrawIcon(apertus_logo_cropped.pixel_data, 0, 0, apertus_logo_cropped.width, apertus_logo_cropped.height, 0x0000, 0xFFFF);

    REQUIRE(painter.MatchBound(1565412, 486524) == true);   // precomputed values
}

