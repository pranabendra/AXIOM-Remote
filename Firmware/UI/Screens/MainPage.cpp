#include "MainPage.h"

#include "../MenuDefinitions.h"
#include "../MenuSystem.h"

MainPage::MainPage(IUSBDevice* cdcDevice) : _usbDevice(cdcDevice), _backgroundColor(Color565::MenuBackground)
{
    SetupButtons();
}

void MainPage::SetupButtons()
{
    // Top row
    _fpsButton = MainPageButton(10, 0, 90, "FPS", true);

    _analogGainButton = MainPageButton(115, 0, 90, "A. Gain", true);
    _analogGainButton.SetHandler(&AnalogGainButtonHandler);

    _digitalGainButton = MainPageButton(220, 0, 90, "D. Gain", true),
    _digitalGainButton.SetHandler(&DigitalGainButtonHandler);

    // Bottom row
    _menuButton = MainPageButton(10, 210, 90, "MENU", false, ButtonType::BUTTON);
    _menuButton.SetLabelHeight(30);
    //_menuButton.HideValue(true);
    _menuButton.SetLabelFont(Font::FreeSans12pt7b);
    _menuButton.SetHandler(&MenuButtonHandler);

    _shutterButton = MainPageButton(115, 179, 90, "Shutter");
    _whiteBalanceButton = MainPageButton(220, 179, 90, "WB");
}

Color565 MainPage::GetBackgroundColor()
{
    return _backgroundColor;
}

void MainPage::SetBackgroundColor(Color565 color)
{
    _backgroundColor = color;
}

void MainPage::MenuButtonHandler(void* sender)
{
    MainPage* menu = static_cast<MainPage*>(sender);
    menu->SetBackgroundColor(Color565::Red);
}

void MainPage::AnalogGainButtonHandler(void* sender)
{
    MainPage* menu = static_cast<MainPage*>(sender);
    menu->SetBackgroundColor(Color565::Green);
}

void MainPage::DigitalGainButtonHandler(void* sender)
{
    MainPage* menu = static_cast<MainPage*>(sender);
    menu->SetBackgroundColor(Color565::MenuBackground);
}

void MainPage::Draw(Painter* painter)
{
    // Diabling the circle in the screen
    // painter->DrawImage(apertus_logo.pixel_data, 58, 89, apertus_logo.width, apertus_logo.height);
    // painter->DrawCircle(100, 100, 20, 0x0);
    
    painter->DrawIcon(apertus_icon_apertus.pixel_data, 52, 83, apertus_icon_apertus.width, apertus_icon_apertus.height, RGB565(177, 173, 166) , 0);
    painter->DrawIcon(apertus_icon_degree.pixel_data, 266, 83, apertus_icon_degree.width, apertus_icon_degree.height, RGB565(250, 135, 86), 0);

    for (uint8_t index = 0; index < 6; index++)
    {
        IWidget* widget = _widgetArray[index];
        if (widget == nullptr)
        {
            return;
        }

        widget->Draw(painter);
    }
}

void MainPage::Update(Button button, int8_t knob, IMenuSystem* menuSystem)
{
    switch (button)
    {
    case Button::BUTTON_1_UP:
        _fpsButton.SetValue((char*)"1U");
        // _usbDevice->Send((uint8_t*)"Button 1 Up\r\n", 10);
        break;
    case Button::BUTTON_1_DOWN:
        _fpsButton.SetValue((char*)"1D");
        //_usbDevice->Send((uint8_t*)"Button 1 Down\r\n", 10);
        break;
    case Button::BUTTON_2_UP:
        _fpsButton.SetValue((char*)"2");
        // _analogGainButton.Activate(this);
        // _usbDevice->Send((uint8_t*)"Button 2\r\n", 10);
        break;
    case Button::BUTTON_3_UP:
        _fpsButton.SetValue((char*)"3");
        //_digitalGainButton.Activate(this);
        //_usbDevice->Send((uint8_t*)"Button 3\r\n", 10);
        break;
    case Button::BUTTON_4_UP:
        _fpsButton.SetValue((char*)"4");
        //_menuButton.Activate(this);
        //_usbDevice->Send((uint8_t*)"Button 4\r\n", 10);

        menuSystem->SetCurrentScreen(AvailableScreens::SettingsMenu);

        break;
    default:
        break;
    }

    /*if (knob < 0)
    {
        _usbDevice->Send((uint8_t*)"Knob \r\n", 10);
    }*/
}
