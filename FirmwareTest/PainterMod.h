#ifndef PAINTERMOD_H
#define PAINTERMOD_H

#include <iostream>

#include "../Firmware/UI/Painter.h"

// For testing the new image
#include "./Media/Images/ApertusLogoCropped.h"

#define FRAMEBUFFER_WIDTH 320
#define FRAMEBUFFER_HEIGHT 240

// Used for checks of proper draw direction, otherwise performance can degrade massively
class PainterMod : public Painter
{
  public:
    uint16_t currentX = 0;
    uint16_t currentY = 0;

    bool wrongDirection = false;

    PainterMod(volatile uint16_t* framebuffer, uint16_t framebufferWidth, uint8_t framebufferHeight) :
        Painter(framebuffer, framebufferWidth, framebufferHeight)
    {
    }

    void DrawPixel(uint16_t x, uint16_t y, uint16_t color) override
    {
        Painter::DrawPixel(x, y, color);

        if(y > currentY)
        {
            currentX = 0;
        }

        if (x < currentX || y < currentY)
        {
            wrongDirection = true;
        }

        currentX = x;
        currentY = y;
    }

    uint16_t GetPixel(uint16_t x, uint16_t y)
    {
        if(x < FRAMEBUFFER_WIDTH && y < FRAMEBUFFER_HEIGHT)
        {
            return _framebuffer[y * FRAMEBUFFER_WIDTH + x];
        }

        std::cout << "Invalid position" << std::endl;
        return 0;
    }

    // Checks whether the icon is 
    bool MatchBound(uint32_t x, uint32_t y)
    {
        return (xWeightedSum == x && yWeightedSum == y);
    }
};

#endif //PAINTERMOD_H
