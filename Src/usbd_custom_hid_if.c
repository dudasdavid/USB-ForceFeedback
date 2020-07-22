/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */ 


0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
0x09, 0x04,                    // USAGE (Joystick)
0xa1, 0x01,                    // COLLECTION (Application)
// Start Joystick input definition
    0x85, 0x01,                //     REPORT_ID (1)
    // Define the butons
    0x05, 0x09,                //     USAGE_PAGE (Button)
    0x15, 0x00,                //     Logical Minimum 0
    0x19, 0x01,                //     USAGE_MINIMUM (Button 1)
    0x29, 0x0A,                //     USAGE_MAXIMUM (Button 10)
    0x25, 0x01,                //     LOGICAL_MAXIMUM (1)
    0x35, 0x00,                //     Physical Minimum 0
    0x45, 0x01,                //     Physical Maximum 1
    0x75, 0x01,                //     REPORT_SIZE (1)
    0x95, 0x0A,                //     REPORT_COUNT (10)
    0x81, 0x02,                //     INPUT (Data,Var,Abs)
    0x95, 0x01,                //     REPORT_COUNT (1)
    0x75, 0x06,                //     REPORT_SIZE (6)
    0x81, 0x03,                //     INPUT (Cnst,Var,Abs)
    // Define the wheel
    0x05, 0x01,                //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                //     USAGE (X)
    0x15, 0x00,                //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x0f,          //     LOGICAL_MAXIMUM (4095)
    0x75, 0x0C,                //     REPORT_SIZE (12)
    0x95, 0x01,                //     REPORT_COUNT (1)
    0x81, 0x02,                //     INPUT (Data,Var,Abs)
    0x75, 0x04,                //     Report Size 4
    0x81, 0x03,                //     Input (Constant, Variable)
    // Define the accelerator and brake
    0xA1, 0x00,                //     Collection Linked
        0x05, 0x01,            //         Usage Page Generic Desktop
        0x09, 0x31,            //         Usage Y
        0x15, 0x00,            //         LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,      //         LOGICAL_MAXIMUM (255)
        0x75, 0x08,            //         REPORT_SIZE (8)
        0x95, 0x01,            //         REPORT_COUNT (1)
        0x81, 0x02,            //         INPUT (Data,Var,Abs)
        0x09, 0x35,            //         Usage Rz
        0x75, 0x08,            //         REPORT_SIZE (8)
        0x81, 0x02,            //         INPUT (Data,Var,Abs)
    0xC0,                      //     End Collection
    // End Joystick Input definition
    0x05,0x0F,       //    Usage Page Physical Interface
    0x09,0x92,       //    Usage ES Playing
    0xA1,0x02,       //    Collection Datalink
       0x85,0x02,    //    Report ID 2
       0x09,0x9F,    //    Usage DS Device is Reset
       0x09,0xA0,    //    Usage DS Device is Pause
       0x09,0xA4,    //    Usage Actuator Power
       0x09,0xA5,    //    Usage Undefined
       0x09,0xA6,    //    Usage Undefined
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x01,    //    Report Size 1
       0x95,0x05,    //    Report Count 5
       0x81,0x02,    //    Input (Variable)
       0x95,0x03,    //    Report Count 3
       0x81,0x03,    //    Input (Constant, Variable)
       0x09,0x94,    //    Usage PID Device Control
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x01,    //    Report Size 1
       0x95,0x01,    //    Report Count 1
       0x81,0x02,    //    Input (Variable)
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x07,    //    Report Size 7
       0x95,0x01,    //    Report Count 1
       0x81,0x02,    //    Input (Variable)
    0xC0    ,    // End Collection
    0x09,0x21,    //    Usage Set Effect Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x01,    //    Report ID 1
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x25,    //    Usage Effect Type
       0xA1,0x02,    //    Collection Datalink
          0x09,0x26,    //    Usage ET Constant Force
          0x09,0x27,    //    Usage ET Ramp
          0x09,0x30,    //    Usage ET Square
          0x09,0x31,    //    Usage ET Sine
          0x09,0x32,    //    Usage ET Triangle
          0x09,0x33,    //    Usage ET Sawtooth Up
          0x09,0x34,    //    Usage ET Sawtooth Down
          0x09,0x40,    //    Usage ET Spring
          0x09,0x41,    //    Usage ET Damper
          0x09,0x42,    //    Usage ET Inertia
          0x09,0x43,    //    Usage ET Friction
          0x09,0x28,    //    Usage ET Custom Force Data
          0x25,0x0C,    //    Logical Maximum Ch (12d)
          0x15,0x01,    //    Logical Minimum 1
          0x35,0x01,    //    Physical Minimum 1
          0x45,0x0C,    //    Physical Maximum Ch (12d)
          0x75,0x08,    //    Report Size 8
          0x95,0x01,    //    Report Count 1
          0x91,0x00,    //    Output
       0xC0    ,          //    End Collection
       0x09,0x50,         //    Usage Duration
       0x09,0x54,         //    Usage Trigger Repeat Interval
       0x09,0x51,         //    Usage Sample Period
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x03,         //    Report Count 3
       0x91,0x02,         //    Output (Variable)
       0x55,0x00,         //    Unit Exponent 0
       0x66,0x00,0x00,    //    Unit 0
       0x09,0x52,         //    Usage Gain
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x53,         //    Usage Trigger Button
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x08,         //    Logical Maximum 8
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x08,         //    Physical Maximum 8
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x55,         //    Usage Axes Enable
       0xA1,0x02,         //    Collection Datalink
          0x05,0x01,    //    Usage Page Generic Desktop
          0x09,0x30,    //    Usage X
          0x09,0x31,    //    Usage Y
          0x15,0x00,    //    Logical Minimum 0
          0x25,0x01,    //    Logical Maximum 1
          0x75,0x01,    //    Report Size 1
          0x95,0x02,    //    Report Count 2
          0x91,0x02,    //    Output (Variable)
       0xC0     ,    // End Collection
       0x05,0x0F,    //    Usage Page Physical Interface
       0x09,0x56,    //    Usage Direction Enable
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x95,0x05,    //    Report Count 5
       0x91,0x03,    //    Output (Constant, Variable)
       0x09,0x57,    //    Usage Direction
       0xA1,0x02,    //    Collection Datalink
          0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
          0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
          0x66,0x14,0x00,              //    Unit 14h (20d)
          0x55,0xFE,                   //    Unit Exponent FEh (254d)
          0x15,0x00,                   //    Logical Minimum 0
          0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
          0x35,0x00,                   //    Physical Minimum 0
          0x47,0xA0,0x8C,0x00,0x00,    //    Physical Maximum 8CA0h (36000d)
          0x66,0x00,0x00,              //    Unit 0
          0x75,0x08,                   //    Report Size 8
          0x95,0x02,                   //    Report Count 2
          0x91,0x02,                   //    Output (Variable)
          0x55,0x00,                   //    Unit Exponent 0
          0x66,0x00,0x00,              //    Unit 0
       0xC0     ,         //    End Collection
       0x05,0x0F,         //    Usage Page Physical Interface
       0x09,0xA7,         //    Usage Undefined
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x66,0x00,0x00,    //    Unit 0
       0x55,0x00,         //    Unit Exponent 0
    0xC0     ,    //    End Collection
    0x05,0x0F,    //    Usage Page Physical Interface
    0x09,0x5A,    //    Usage Set Envelope Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x02,         //    Report ID 2
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x5B,         //    Usage Attack Level
       0x09,0x5D,         //    Usage Fade Level
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x09,0x5C,         //    Usage Attack Time
       0x09,0x5E,         //    Usage Fade Time
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x91,0x02,         //    Output (Variable)
       0x45,0x00,         //    Physical Maximum 0
       0x66,0x00,0x00,    //    Unit 0
       0x55,0x00,         //    Unit Exponent 0
    0xC0     ,            //    End Collection
    0x09,0x5F,    //    Usage Set Condition Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x03,    //    Report ID 3
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x23,    //    Usage Parameter Block Offset
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x04,    //    Report Size 4
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x58,    //    Usage Type Specific Block Off...
       0xA1,0x02,    //    Collection Datalink
          0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
          0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
          0x75,0x02,                   //    Report Size 2
          0x95,0x02,                   //    Report Count 2
          0x91,0x02,                   //    Output (Variable)
       0xC0     ,         //    End Collection
       0x15,0x80,         //    Logical Minimum 80h (-128d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x60,         //    Usage CP Offset
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x61,         //    Usage Positive Coefficient
       0x09,0x62,         //    Usage Negative Coefficient
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x63,         //    Usage Positive Saturation
       0x09,0x64,         //    Usage Negative Saturation
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x09,0x65,         //    Usage Dead Band
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x6E,    //    Usage Set Periodic Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x04,                   //    Report ID 4
       0x09,0x22,                   //    Usage Effect Block Index
       0x15,0x01,                   //    Logical Minimum 1
       0x25,0x28,                   //    Logical Maximum 28h (40d)
       0x35,0x01,                   //    Physical Minimum 1
       0x45,0x28,                   //    Physical Maximum 28h (40d)
       0x75,0x08,                   //    Report Size 8
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x70,                   //    Usage Magnitude
       0x15,0x00,                   //    Logical Minimum 0
       0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
       0x35,0x00,                   //    Physical Minimum 0
       0x46,0x10,0x27,              //    Physical Maximum 2710h (10000d)
       0x75,0x08,                   //    Report Size 8
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x6F,                   //    Usage Offset
       0x15,0x80,                   //    Logical Minimum 80h (-128d)
       0x25,0x7F,                   //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,              //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,              //    Physical Maximum 2710h (10000d)
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x71,                   //    Usage Phase
       0x66,0x14,0x00,              //    Unit 14h (20d)
       0x55,0xFE,                   //    Unit Exponent FEh (254d)
       0x15,0x00,                   //    Logical Minimum 0
       0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
       0x35,0x00,                   //    Physical Minimum 0
       0x47,0xA0,0x8C,0x00,0x00,    //    Physical Maximum 8CA0h (36000d)
       0x91,0x02,                   //    Output (Variable)
       0x09,0x72,                   //    Usage Period
       0x26,0xFF,0x7F,              //    Logical Maximum 7FFFh (32767d)
       0x46,0xFF,0x7F,              //    Physical Maximum 7FFFh (32767d)
       0x66,0x03,0x10,              //    Unit 1003h (4099d)
       0x55,0xFD,                   //    Unit Exponent FDh (253d)
       0x75,0x10,                   //    Report Size 10h (16d)
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x66,0x00,0x00,              //    Unit 0
       0x55,0x00,                   //    Unit Exponent 0
    0xC0     ,    // End Collection
    0x09,0x73,    //    Usage Set Constant Force Rep...
    0xA1,0x02,    //    Collection Datalink
       0x85,0x05,         //    Report ID 5
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x70,         //    Usage Magnitude
       0x16,0x01,0xFF,    //    Logical Minimum FF01h (-255d)
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x74,    //    Usage Set Ramp Force Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x06,         //    Report ID 6
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x75,         //    Usage Ramp Start
       0x09,0x76,         //    Usage Ramp End
       0x15,0x80,         //    Logical Minimum 80h (-128d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x68,    //    Usage Custom Force Data Rep...
    0xA1,0x02,    //    Collection Datalink
       0x85,0x07,         //    Report ID 7
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x6C,         //    Usage Custom Force Data Offset
       0x15,0x00,         //    Logical Minimum 0
       0x26,0x10,0x27,    //    Logical Maximum 2710h (10000d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x69,         //    Usage Custom Force Data
       0x15,0x81,         //    Logical Minimum 81h (-127d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x0C,         //    Report Count Ch (12d)
       0x92,0x02,0x01,    //       Output (Variable, Buffered)
    0xC0     ,    //    End Collection
    0x09,0x66,    //    Usage Download Force Sample
    0xA1,0x02,    //    Collection Datalink
       0x85,0x08,         //    Report ID 8
       0x05,0x01,         //    Usage Page Generic Desktop
       0x09,0x30,         //    Usage X
       0x09,0x31,         //    Usage Y
       0x15,0x81,         //    Logical Minimum 81h (-127d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
    0xC0     ,   //    End Collection
    0x05,0x0F,   //    Usage Page Physical Interface
    0x09,0x77,   //    Usage Effect Operation Report
    0xA1,0x02,   //    Collection Datalink
       0x85,0x0A,    //    Report ID Ah (10d)
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x78,    //    Usage Operation
       0xA1,0x02,    //    Collection Datalink
          0x09,0x79,    //    Usage Op Effect Start
          0x09,0x7A,    //    Usage Op Effect Start Solo
          0x09,0x7B,    //    Usage Op Effect Stop
          0x15,0x01,    //    Logical Minimum 1
          0x25,0x03,    //    Logical Maximum 3
          0x75,0x08,    //    Report Size 8
          0x95,0x01,    //    Report Count 1
          0x91,0x00,    //    Output
       0xC0     ,         //    End Collection
       0x09,0x7C,         //    Usage Loop Count
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x90,    //    Usage PID State Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0B,    //    Report ID Bh (11d)
       0x09,0x22,    //    Usage Effect Block Index
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x96,    //    Usage DC Disable Actuators
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0C,    //    Report ID Ch (12d)
       0x09,0x97,    //    Usage DC Stop All Effects
       0x09,0x98,    //    Usage DC Device Reset
       0x09,0x99,    //    Usage DC Device Pause
       0x09,0x9A,    //    Usage DC Device Continue
       0x09,0x9B,    //    Usage PID Device State
       0x09,0x9C,    //    Usage DS Actuators Enabled
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x06,    //    Logical Maximum 6
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x00,    //    Output
    0xC0     ,    //    End Collection
    0x09,0x7D,    //    Usage PID Pool Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0D,         //    Report ID Dh (13d)
       0x09,0x7E,         //    Usage RAM Pool Size
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,            //    End Collection
    0x09,0x6B,    //    Usage Set Custom Force Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0E,         //    Report ID Eh (14d)
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x6D,         //    Usage Sample Count
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x51,         //    Usage Sample Period
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x55,0x00,         //    Unit Exponent 0
       0x66,0x00,0x00,    //    Unit 0
    0xC0     ,    //    End Collection
    0x09,0xAB,    //    Usage Undefined
    0xA1,0x02,    //    Collection Datalink
       0x85,0x01,    //    Report ID 1
       0x09,0x25,    //    Usage Effect Type
       0xA1,0x02,    //    Collection Datalink
       0x09,0x26,    //    Usage ET Constant Force
       0x09,0x27,    //    Usage ET Ramp
       0x09,0x30,    //    Usage ET Square
       0x09,0x31,    //    Usage ET Sine
       0x09,0x32,    //    Usage ET Triangle
       0x09,0x33,    //    Usage ET Sawtooth Up
       0x09,0x34,    //    Usage ET Sawtooth Down
       0x09,0x40,    //    Usage ET Spring
       0x09,0x41,    //    Usage ET Damper
       0x09,0x42,    //    Usage ET Inertia
       0x09,0x43,    //    Usage ET Friction
       0x09,0x28,    //    Usage ET Custom Force Data
       0x25,0x0C,    //    Logical Maximum Ch (12d)
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x0C,    //    Physical Maximum Ch (12d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0xB1,0x00,    //    Feature
    0xC0     ,    // End Collection
    0x05,0x01,         //    Usage Page Generic Desktop
    0x09,0x3B,         //    Usage Byte Count
    0x15,0x00,         //    Logical Minimum 0
    0x26,0xFF,0x01,    //    Logical Maximum 1FFh (511d)
    0x35,0x00,         //    Physical Minimum 0
    0x46,0xFF,0x01,    //    Physical Maximum 1FFh (511d)
    0x75,0x0A,         //    Report Size Ah (10d)
    0x95,0x01,         //    Report Count 1
    0xB1,0x02,         //    Feature (Variable)
    0x75,0x06,         //    Report Size 6
    0xB1,0x01,         //    Feature (Constant)
 0xC0     ,    //    End Collection
 0x05,0x0F,    //    Usage Page Physical Interface
 0x09,0x89,    //    Usage Block Load Status
 0xA1,0x02,    //    Collection Datalink
    0x85,0x02,    //    Report ID 2
    0x09,0x22,    //    Usage Effect Block Index
    0x25,0x28,    //    Logical Maximum 28h (40d)
    0x15,0x01,    //    Logical Minimum 1
    0x35,0x01,    //    Physical Minimum 1
    0x45,0x28,    //    Physical Maximum 28h (40d)
    0x75,0x08,    //    Report Size 8
    0x95,0x01,    //    Report Count 1
    0xB1,0x02,    //    Feature (Variable)
    0x09,0x8B,    //    Usage Block Load Full
    0xA1,0x02,    //    Collection Datalink
       0x09,0x8C,    //    Usage Block Load Error
       0x09,0x8D,    //    Usage Block Handle
       0x09,0x8E,    //    Usage PID Block Free Report
       0x25,0x03,    //    Logical Maximum 3
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x03,    //    Physical Maximum 3
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0xB1,0x00,    //    Feature
    0xC0     ,                   // End Collection
    0x09,0xAC,                   //    Usage Undefined
    0x15,0x00,                   //    Logical Minimum 0
    0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
    0x35,0x00,                   //    Physical Minimum 0
    0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
    0x75,0x10,                   //    Report Size 10h (16d)
    0x95,0x01,                   //    Report Count 1
    0xB1,0x00,                   //    Feature
 0xC0     ,    //    End Collection
 0x09,0x7F,    //    Usage ROM Pool Size
 0xA1,0x02,    //    Collection Datalink
    0x85,0x03,                   //    Report ID 3
    0x09,0x80,                   //    Usage ROM Effect Block Count
    0x75,0x10,                   //    Report Size 10h (16d)
    0x95,0x01,                   //    Report Count 1
    0x15,0x00,                   //    Logical Minimum 0
    0x35,0x00,                   //    Physical Minimum 0
    0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
    0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
    0xB1,0x02,                   //    Feature (Variable)
    0x09,0x83,                   //    Usage PID Pool Move Report
    0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
    0x46,0xFF,0x00,              //    Physical Maximum FFh (255d)
    0x75,0x08,                   //    Report Size 8
    0x95,0x01,                   //    Report Count 1
    0xB1,0x02,                   //    Feature (Variable)
    0x09,0xA9,                   //    Usage Undefined
    0x09,0xAA,                   //    Usage Undefined
    0x75,0x01,                   //    Report Size 1
    0x95,0x02,                   //    Report Count 2
    0x15,0x00,                   //    Logical Minimum 0
    0x25,0x01,                   //    Logical Maximum 1
    0x35,0x00,                   //    Physical Minimum 0
    0x45,0x01,                   //    Physical Maximum 1
    0xB1,0x02,                   //    Feature (Variable)
    0x75,0x06,                   //    Report Size 6
    0x95,0x01,                   //    Report Count 1
    0xB1,0x03,                   //    Feature (Constant, Variable)
    0xC0,    //    End Collection
 //0xC0    //    End Collection
 


  
  /* USER CODE END 0 */ 
  0xC0    /*     END_COLLECTION	             */
   
}; 
/* USB handler declaration */
/* Handle for USB Full Speed IP */
USBD_HandleTypeDef  *hUsbDevice_0;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t disFlag = 0;

/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static int8_t CUSTOM_HID_Init_FS     (void);
static int8_t CUSTOM_HID_DeInit_FS   (void);
static int8_t CUSTOM_HID_OutEvent_FS (uint8_t event_idx, uint8_t* buffer);
/* USER CODE BEGIN 2 */ 
static int8_t CUSTOM_HID_SetFeature_FS  (uint8_t event_idx, uint8_t* buffer);
static int8_t CUSTOM_HID_GetFeature_FS  (uint8_t event_idx, uint8_t* buffer, uint8_t report_len, uint16_t* length);

extern uint32_t lastMessageTime;
extern uint32_t uwTick;
extern uint32_t cfStart;
extern uint16_t cfDuration;
extern uint16_t cfDelay;
extern uint8_t cfDirection;
extern int16_t cfConstant;
extern uint8_t constantForceEna;

extern uint8_t AnswerFlag;
extern uint8_t ReceiveFlag;
extern uint8_t sendBusy;

extern uint8_t buffer2[3];
/* USER CODE END 2 */ 

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = 
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
  CUSTOM_HID_SetFeature_FS,
  CUSTOM_HID_GetFeature_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CUSTOM_HID_Init_FS
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  hUsbDevice_0 = &hUsbDeviceFS;
  /* USER CODE BEGIN 4 */ 
  return (0);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CUSTOM_HID_DeInit_FS
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */ 
  return (0);
  /* USER CODE END 5 */ 
}

/**
  * @brief  CUSTOM_HID_OutEvent_FS
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS  (uint8_t event_idx, uint8_t* buffer)
{ 
  /* USER CODE BEGIN 6 */ 
  
  lastMessageTime = uwTick;
  sendBusy = 1;
  uint8_t i;
  uint8_t len = 16;
  //printf("OutEvent: %02X - ", event_idx);
  //printf("Out Report (id:%X): ", event_idx);
  /*
  printf("\r\n");
  printf("XXXXXXXX\r\n");
  for (i = 0; i < 16; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
  printf("\r\n");
  */
  //ReceiveFlag=1;
   switch(event_idx)
   {
      case 1: // Out Report 1
        len = 16;
        /*
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        */
        
        if (buffer[2] == 0x01) {
        
          cfDuration = buffer[3] + (buffer[4]<<8);
          cfDelay = buffer[14] + (buffer[15]<<8);
          cfDirection = buffer[12];
        }
          
        break;
      case 2: // Out Report 2
        len = 8;
        
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;
        
      case 5: // Out Report 5
        
        //USB_MASK_INTERRUPT((&hpcd_USB_OTG_FS)->Instance, USB_OTG_GINTSTS_IEPINT);
        len = 4;
        //disFlag = 1;
        /*
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        */
        //AnswerFlag = 1;
        cfConstant = buffer[2] + (buffer[3]<<8);
        //printf("%i", cfConstant);
        break;
        
      case 10: // Out Report 10 (0x0A)
        len = 4;
        disFlag = 0;
        /*
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        */
        AnswerFlag = 1;
        if (buffer[2] == 0x03){
          constantForceEna = 0;
        }
        else if (buffer[2] == 0x01){
          cfStart = uwTick;
          constantForceEna = 1;         
        }
        
        //USB_UNMASK_INTERRUPT((&hpcd_USB_OTG_FS)->Instance, USB_OTG_GINTSTS_IEPINT);
        
        //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,buffer2,3);
        
        break;
        
      case 11: // Out Report 11 (0x0B)
        len = 2;
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;
        
      case 12: // Out Report 12 (0x0C)
        len = 2;
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;
        
      case 13: // Out Report 13 (0x0D)
        len = 2;
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;

      default: /* Report does not exist */
        printf("DEFAULT!");
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;
   }
  
  

  
  return (0);
  /* USER CODE END 6 */ 
}

/* USER CODE BEGIN 7 */ 

static int8_t CUSTOM_HID_SetFeature_FS(uint8_t event_idx, uint8_t* buffer)
{
  
  uint8_t i;
  uint8_t len = 16;
  printf("Set Report (Feature id:%X): ", event_idx);
  
   switch(event_idx)
   {
      case 1:
        len = 4;
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");
        break;


      default: /* Report does not exist */
        printf("DEFAULT!");
        for (i = 0; i < len; i++)
        {
          if (i > 0) printf(",");
          printf("%02X", buffer[i]);
        }
        printf("\r\n");

         break;
   }
 
   return (USBD_OK);
 
}
 
 
 
static int8_t CUSTOM_HID_GetFeature_FS(uint8_t event_idx, uint8_t* buffer, uint8_t report_len, uint16_t* length)
{
   uint8_t* data;
   uint16_t len;
   data = &buffer[1];         // as ReportID must reside inside the first byte
   
  printf("Get Report (Feature id:%X, Length:%i)\r\n", event_idx, report_len);
   
  
  
   switch(event_idx)
   {
      case 1: /* LED1 */
         data[0]=event_idx+0x10;
         len = 1;
//         (state == 1) ? BSP_LED_On(LED1) : BSP_LED_Off(LED1);
         break;
 
      case 2: /* LED2 */
        buffer[1] = 0x02;
        buffer[2] = 0x01;
        buffer[3] = 0xD9;
        buffer[4] = 0x04;
        len = 4;
        break;
 
      case 3: /* LED3 */
        buffer[1] = 0x14;
        buffer[2] = 0x05;
        buffer[3] = 0x0F;
        buffer[4] = 0x01;
        len = 4;

        break;
 
 
      default: /* Report does not exist */
         return (USBD_FAIL);
         break;
   }
   
   buffer[0] = event_idx;  // ReportID must reside inside the first byte
   *length = len + 1;      // +1 -> ReportID must also be considered


   return (USBD_OK);
}



/**
  * @brief  USBD_CUSTOM_HID_SendReport_FS
  *         Send the report to the Host       
  * @param  report: the report to be sent
  * @param  len: the report length
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
/*  
static int8_t USBD_CUSTOM_HID_SendReport_FS ( uint8_t *report,uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(hUsbDevice_0, report, len); 
}
*/
/* USER CODE END 7 */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
