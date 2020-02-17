//*****************************************************************************
//
// adc.c - Driver for the ADC.
//
// Copyright (c) 2005-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.0.1.11577 of the Tiva Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup adc_api
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "adc.h"
#include "debug.h"
#include "interrupt.h"

//*****************************************************************************
//
// These defines are used by the ADC driver to simplify access to the ADC
// sequencer's registers.
//
//*****************************************************************************
#define ADC_SEQ                 (ADC_O_SSMUX0)
#define ADC_SEQ_STEP            (ADC_O_SSMUX1 - ADC_O_SSMUX0)
#define ADC_SSMUX               (ADC_O_SSMUX0 - ADC_O_SSMUX0)
#define ADC_SSEMUX              (ADC_O_SSEMUX0 - ADC_O_SSMUX0)
#define ADC_SSCTL               (ADC_O_SSCTL0 - ADC_O_SSMUX0)
#define ADC_SSFIFO              (ADC_O_SSFIFO0 - ADC_O_SSMUX0)
#define ADC_SSFSTAT             (ADC_O_SSFSTAT0 - ADC_O_SSMUX0)
#define ADC_SSOP                (ADC_O_SSOP0 - ADC_O_SSMUX0)
#define ADC_SSDC                (ADC_O_SSDC0 - ADC_O_SSMUX0)

//*****************************************************************************
//
// The currently configured software oversampling factor for each of the ADC
// sequencers.
//
//*****************************************************************************
static uint8_t g_pui8OversampleFactor[3];

//*****************************************************************************
//
//! Returns the interrupt number for a given ADC base address and sequence
//! number.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function returns the interrupt number for the ADC module and sequence
//! number provided in the \e ui32Base and \e ui32SequenceNum parameters.
//!
//! \return Returns the ADC sequence interrupt number or 0 if the interrupt
//! does not exist.
//
//*****************************************************************************
static uint_fast8_t
_ADCIntNumberGet(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    uint_fast8_t ui8Int;

    //
    // Determine the interrupt to register based on the sequence number.
    //
    if(CLASS_IS_BLIZZARD)
    {
        ui8Int = ((ui32Base == ADC0_BASE) ?
                  (INT_ADC0SS0_BLIZZARD + ui32SequenceNum) :
                  (INT_ADC0SS0_BLIZZARD + ui32SequenceNum));
    }
    else if(CLASS_IS_SNOWFLAKE)
    {
        ui8Int = ((ui32Base == ADC0_BASE) ?
                  (INT_ADC0SS0_SNOWFLAKE + ui32SequenceNum) :
                  (INT_ADC1SS0_SNOWFLAKE + ui32SequenceNum));
    }
    else
    {
        ui8Int = 0;
    }

    return(ui8Int);
}

//*****************************************************************************
//
//! Registers an interrupt handler for an ADC interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//! \param pfnHandler is a pointer to the function to be called when the
//! ADC sample sequence interrupt occurs.
//!
//! This function sets the handler to be called when a sample sequence
//! interrupt occurs.  This function enables the global interrupt in the
//! interrupt controller; the sequence interrupt must be enabled with
//! ADCIntEnable().  It is the interrupt handler's responsibility to clear the
//! interrupt source via ADCIntClear().
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntRegister(uint32_t ui32Base, uint32_t ui32SequenceNum,
               void (*pfnHandler)(void))
{
    uint_fast8_t ui8Int;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Determine the interrupt to register based on the sequence number.
    //
    ui8Int = _ADCIntNumberGet(ui32Base, ui32SequenceNum);
    ASSERT(ui8Int != 0);

    //
    // Register the interrupt handler.
    //
    IntRegister(ui8Int, pfnHandler);

    //
    // Enable the timer interrupt.
    //
    IntEnable(ui8Int);
}

//*****************************************************************************
//
//! Unregisters the interrupt handler for an ADC interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function unregisters the interrupt handler.  This function disables
//! the global interrupt in the interrupt controller; the sequence interrupt
//! must be disabled via ADCIntDisable().
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntUnregister(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    uint_fast8_t ui8Int;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Determine the interrupt to unregister based on the sequence number.
    //
    ui8Int = _ADCIntNumberGet(ui32Base, ui32SequenceNum);
    ASSERT(ui8Int != 0);

    //
    // Disable the interrupt.
    //
    IntDisable(ui8Int);

    //
    // Unregister the interrupt handler.
    //
    IntUnregister(ui8Int);
}

//*****************************************************************************
//
//! Disables a sample sequence interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function disables the requested sample sequence interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntDisable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Disable this sample sequence interrupt.
    //
    HWREG(ui32Base + ADC_O_IM) &= ~(1 << ui32SequenceNum);
}

//*****************************************************************************
//
//! Enables a sample sequence interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function enables the requested sample sequence interrupt.  Any
//! outstanding interrupts are cleared before enabling the sample sequence
//! interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntEnable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Clear any outstanding interrupts on this sample sequence.
    //
    HWREG(ui32Base + ADC_O_ISC) = 1 << ui32SequenceNum;

    //
    // Enable this sample sequence interrupt.
    //
    HWREG(ui32Base + ADC_O_IM) |= 1 << ui32SequenceNum;
}

//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//! \param bMasked is false if the raw interrupt status is required and true if
//! the masked interrupt status is required.
//!
//! This function returns the interrupt status for the specified sample
//! sequence.  Either the raw interrupt status or the status of interrupts that
//! are allowed to reflect to the processor can be returned.
//!
//! \return The current raw or masked interrupt status.
//
//*****************************************************************************

/*******************************************************************************
** 函数名称: ADCIntStatus
** 功能描述: 获取当前中断状态 
** 参数说明: ui32Base 是 ADC 模块的基址。
**			 ui32SequenceNum 是样本序列号
**			 bMasked 如果需 要原始中断状态，则为 false；如果需要屏蔽中断状态，则为 true。 
** 返回说明: 当前的原始或屏蔽中断状态 
** 描    述: 此函数返回指定示例序列的中断状态。允许向处理器反映的原始中断状态或中断状态可以被归还   
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
uint32_t
ADCIntStatus(uint32_t ui32Base, uint32_t ui32SequenceNum, bool bMasked)
{
    uint32_t ui32Temp;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Return either the interrupt status or the raw interrupt status as
    // requested.
    //
    if(bMasked)
    {
        ui32Temp = HWREG(ui32Base + ADC_O_ISC) & (0x10001 << ui32SequenceNum);
    }
    else
    {
        ui32Temp = (HWREG(ui32Base + ADC_O_RIS) &
                    (0x10000 | (1 << ui32SequenceNum)));

        //
        // If the digital comparator status bit is set, reflect it to the
        // appropriate sequence bit.
        //
        if(ui32Temp & 0x10000)
        {
            ui32Temp |= 0xF0000;
            ui32Temp &= ~(0x10000 << ui32SequenceNum);
        }
    }

    //
    // Return the interrupt status
    //
    return(ui32Temp);
}

/*******************************************************************************
** 函数名称: ADCIntClear
** 功能描述: 清除样本序列中断源。 
** 参数说明: ui32Base: [输入/出] 
**			 ui32SequenceNum: [输入/出] 
** 返回说明: None
** 描    述: 指定的示例序列中断被清除，因此不再持续。必须在中断处理程序中调用此函数，
**           以防止中断在退出时立即再次触发。
** 注    意: 请在中断发生第一时刻去清除中断源 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
ADCIntClear(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Clear the interrupt.
    //
    HWREG(ui32Base + ADC_O_ISC) = 1 << ui32SequenceNum;
}

/*******************************************************************************
** 函数名称: ADCSequenceEnable
** 功能描述: 使能 ADC 采集序列 
** 参数说明: ui32Base是 ADC 模块的基址。
**			 ui32SequenceNum 是样本序列号。
** 返回说明: None
** 描    述: 允许在检测到触发器时捕获指定的示例序列。在启用示例序列之前，必须对其进行配置。  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
ADCSequenceEnable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Enable the specified sequence.
    //
    HWREG(ui32Base + ADC_O_ACTSS) |= 1 << ui32SequenceNum;
}

//*****************************************************************************
//
//! Disables a sample sequence.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! Prevents the specified sample sequence from being captured when its trigger
//! is detected.  A sample sequence should be disabled before it is configured.
//!
//! \return None.
//
//*****************************************************************************
void
ADCSequenceDisable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Disable the specified sequences.
    //
    HWREG(ui32Base + ADC_O_ACTSS) &= ~(1 << ui32SequenceNum);
}

/*******************************************************************************
** 函数名称: ADCSequenceConfigure
** 功能描述: 配置示例序列的触发源和优先级。 
** 参数说明: ui32Base是 ADC 模块的基址
**			 ui32SequenceNum是样本序列号
**			 ui32Trigger是启动样本序列的触发源；必须是 ADC_TRIGGER_*值之一
**			 ui32Priority 是样本序列相对于其他样本序列的相对优先级。 
** 返回说明: None
** 描    述：此函数配置示例序列的起始条件。有效样品排序从 0 到 3；排序器0 
**           捕获最多 8 个样 本，一个和两个捕获最多四个样本，而排序器三个
**           捕获一个样本。设置触发条件和优先级(相对于其他示例排序器的执行)。
**           ui32Trigger参数可以采用以下值 
**					 ADC_TRIGGER_PROCESSOR -处理器通过 ADCProcessorTrigger()函数生成的触发器 
**					 ADC_TRIGGER_COMP0 -由第一模拟比较器生成的触发器；用 ComparatorConfigure() 配置 
**					 ADC_TRIGGER_COMP1 -由第二模拟比较器生成的触发器；用 ComparatorConfigure() 配置 
**					 ADC_TRIGGER_COMP2 -由第三模拟比较器生成的触发器；用 ComparatorConfigure() 配置 
**					 ADC_TRIGGER_EXTERNAL - 由端口 B4 引脚的输入产生的触发器。请注意，一些微控制器可
**					 以使用GPIOADCTriggerEnable()函数从任何GPIO中选择
**					 ADC_TRIGGER_TIMER - 由计时器生成的触发器；配置为 TimerControlTrigger() 
**					 ADC_TRIGGER_PWM0 - 由第一个 PWM 生成器生成的触发器；配置为 
**					 PWMGenIntTrigEnable() 
**					 ADC_TRIGGER_PWM1 - 由第二个 PWM 生成器生成的触发器；配置为 
**					 PWMGenIntTrigEnable() 
**					 ADC_TRIGGER_PWM2 - 由第三个 PWM 生成器生成的触发器；配置为 
**					 PWMGenIntTrigEnable() 
**					 ADC_TRIGGER_PWM3 - 由第四个 PWM 生成器生成的触发器；配置为 
**					 PWMGenIntTrigEnable() 
**					 ADC_TRIGGER_ALWAYS - 一个总是被持续的触发器，导致采样序列重复捕获(只要没有更
**					 高的优先级源活动)。
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
ADCSequenceConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum,
											uint32_t ui32Trigger, uint32_t ui32Priority)
{
    //
    // Check the arugments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);
    ASSERT((ui32Trigger == ADC_TRIGGER_PROCESSOR) ||
           (ui32Trigger == ADC_TRIGGER_COMP0) ||
           (ui32Trigger == ADC_TRIGGER_COMP1) ||
           (ui32Trigger == ADC_TRIGGER_COMP2) ||
           (ui32Trigger == ADC_TRIGGER_EXTERNAL) ||
           (ui32Trigger == ADC_TRIGGER_TIMER) ||
           (ui32Trigger == ADC_TRIGGER_PWM0) ||
           (ui32Trigger == ADC_TRIGGER_PWM1) ||
           (ui32Trigger == ADC_TRIGGER_PWM2) ||
           (ui32Trigger == ADC_TRIGGER_PWM3) ||
           (ui32Trigger == ADC_TRIGGER_ALWAYS));
    ASSERT(ui32Priority < 4);

    //
    // Compute the shift for the bits that control this sample sequence.
    //
    ui32SequenceNum *= 4;

    //
    // Set the trigger event for this sample sequence.
    //
    HWREG(ui32Base + ADC_O_EMUX) = ((HWREG(ui32Base + ADC_O_EMUX) &
                                     ~(0xf << ui32SequenceNum)) |
                                    ((ui32Trigger & 0xf) << ui32SequenceNum));

    //
    // Set the priority for this sample sequence.
    //
    HWREG(ui32Base + ADC_O_SSPRI) = ((HWREG(ui32Base + ADC_O_SSPRI) &
                                      ~(0xf << ui32SequenceNum)) |
                                     ((ui32Priority & 0x3) <<
                                      ui32SequenceNum));
}

/*******************************************************************************
** 函数名称: ADCSequenceStepConfigure
** 功能描述: 配置示例排序器的步骤。 
** 参数说明: ui32Base 是 ADC 模块的基址。
**			 ui32SequenceNum 是样本序列号
**			 ui32Step 是要配置的步骤。
**			 ui32Config 是该步骤的配置
**       必须是逻辑或 ADC_CTL_TS，ADC_CTL_IE，ADC_CTL_END， ADC_CTL_D， 
**       输入通道之一选择(ADC_CTL_CH0 到 ADC_CTL_CH 23)，
**       其中一个数字比较器选择(ADC_CTL_CMP0 到 ADC_CTL_CMP7). 
** 返回说明: None
** 描    述: 此函数将 ADC 配置为示例序列的一个步骤。 
**           可以配置为单端或差分操作(ADC_CTL_D 位在设置时选择差分操作) 
**           可以选择要采样的通道（ADC_CTL_CH0 到 ADC_CTL_CH 23），
**           还有内部温度传感器可以 选择（ADC_CTL_TS）。 
**           此外，此步骤可以定义为序列中的最后一步 (ADC_CTL_END) 
**           并且可以将其配置为在步骤 完成时造成中断(ADC_CTL_IE)。 
**           如果设备上存在数字比较器，则还可以配置此步骤，此步骤还可以配置 ADC_CTL_CMP0 
**           到 ADC_CTL_CMP7 为将 adc 示例发送到所选的比较器。当发生此序列的触发器时，
**           adc 将使用此配置。
** 注    意: 如果使用 ADC_CTL_CMP 0 到 ADC_CTL_CMP 7 选择显示并启用了数字比较器，
**           则 ADC 样本不会写入 ADC 序列数据 FIFO 中。 
**					 ui32Step 参数确定触发器发生时由 adc 捕获样本的顺序。对于第一个样品排序器，
**					 它可以从0到7个，对于第二个和第三个样本排序器，从0到3个不等。
**					 而对于第四个样品排序器，只能为0。 
**           差分模式仅适用于相邻的信道对(例如，0 和 1)。通道选择必须是要示例的通道对
**					 的数目。(例如，ADC_CTL_CH0 用于 0 和 1，或 ADC_CTL_CH1 用于 2 和 3) 或由 
**					 adc 返回未定义的结果。此外，如果在采样温度传感器时选择了差分模式，则 adc 
**					 将返回未定义的结果。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
ADCSequenceStepConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum,
                         uint32_t ui32Step, uint32_t ui32Config)
{
    uint32_t ui32Temp;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);
    ASSERT(((ui32SequenceNum == 0) && (ui32Step < 8)) ||
           ((ui32SequenceNum == 1) && (ui32Step < 4)) ||
           ((ui32SequenceNum == 2) && (ui32Step < 4)) ||
           ((ui32SequenceNum == 3) && (ui32Step < 1)));

    //
    // Get the offset of the sequence to be configured.
    //
    ui32Base += ADC_SEQ + (ADC_SEQ_STEP * ui32SequenceNum);

    //
    // Compute the shift for the bits that control this step.
    //
    ui32Step *= 4;

    //
    // Set the analog mux value for this step.
    //
    HWREG(ui32Base + ADC_SSMUX) = ((HWREG(ui32Base + ADC_SSMUX) &
                                    ~(0x0000000f << ui32Step)) |
                                   ((ui32Config & 0x0f) << ui32Step));

    //
    // Set the upper bits of the analog mux value for this step.
    //
    HWREG(ui32Base + ADC_SSEMUX) = ((HWREG(ui32Base + ADC_SSEMUX) &
                                     ~(0x0000000f << ui32Step)) |
                                    (((ui32Config & 0xf00) >> 8) << ui32Step));

    //
    // Set the control value for this step.
    //
    HWREG(ui32Base + ADC_SSCTL) = ((HWREG(ui32Base + ADC_SSCTL) &
                                    ~(0x0000000f << ui32Step)) |
                                   (((ui32Config & 0xf0) >> 4) << ui32Step));

    //
    // Enable digital comparator if specified in the ui32Config bit-fields.
    //
    if(ui32Config & 0x000F0000)
    {
        //
        // Program the comparator for the specified step.
        //
        ui32Temp = HWREG(ui32Base + ADC_SSDC);
        ui32Temp &= ~(0xF << ui32Step);
        ui32Temp |= (((ui32Config & 0x00070000) >> 16) << ui32Step);
        HWREG(ui32Base + ADC_SSDC) = ui32Temp;

        //
        // Enable the comparator.
        //
        HWREG(ui32Base + ADC_SSOP) |= (1 << ui32Step);
    }

    //
    // Disable digital comparator if not specified.
    //
    else
    {
        HWREG(ui32Base + ADC_SSOP) &= ~(1 << ui32Step);
    }
}

//*****************************************************************************
//
//! Determines if a sample sequence overflow occurred.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function determines if a sample sequence overflow has occurred.
//! Overflow happens if the captured samples are not read from the FIFO before
//! the next trigger occurs.
//!
//! \return Returns zero if there was not an overflow, and non-zero if there
//! was.
//
//*****************************************************************************
int32_t
ADCSequenceOverflow(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Determine if there was an overflow on this sequence.
    //
    return(HWREG(ui32Base + ADC_O_OSTAT) & (1 << ui32SequenceNum));
}

//*****************************************************************************
//
//! Clears the overflow condition on a sample sequence.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function clears an overflow condition on one of the sample sequences.
//! The overflow condition must be cleared in order to detect a subsequent
//! overflow condition (it otherwise causes no harm).
//!
//! \return None.
//
//*****************************************************************************
void
ADCSequenceOverflowClear(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Clear the overflow condition for this sequence.
    //
    HWREG(ui32Base + ADC_O_OSTAT) = 1 << ui32SequenceNum;
}

//*****************************************************************************
//
//! Determines if a sample sequence underflow occurred.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function determines if a sample sequence underflow has occurred.
//! Underflow happens if too many samples are read from the FIFO.
//!
//! \return Returns zero if there was not an underflow, and non-zero if there
//! was.
//
//*****************************************************************************
int32_t
ADCSequenceUnderflow(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Determine if there was an underflow on this sequence.
    //
    return(HWREG(ui32Base + ADC_O_USTAT) & (1 << ui32SequenceNum));
}

//*****************************************************************************
//
//! Clears the underflow condition on a sample sequence.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function clears an underflow condition on one of the sample
//! sequencers.  The underflow condition must be cleared in order to detect a
//! subsequent underflow condition (it otherwise causes no harm).
//!
//! \return None.
//
//*****************************************************************************
void
ADCSequenceUnderflowClear(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Clear the underflow condition for this sequence.
    //
    HWREG(ui32Base + ADC_O_USTAT) = 1 << ui32SequenceNum;
}


/*******************************************************************************
** 函数名称: ADCSequenceDataGet
** 功能描述: 获取样本序列的捕获数据
** 参数说明: ui32Base是 ADC 模块的基地址
**			 ui32SequenceNum 是样本序列号
**			 pui32Buffer是 地址的数据存储。(注意这里是地址数据！也就是说需要传一个指针当入口参数)
** 返回说明: 返回复制到缓冲区的样本数。 
** 描    述: 此函数将数据从指定的示例排序器输出 FIFO 复制到内存驻留缓冲区。硬件 
**           FIFO 中可用的样本数被复制到缓冲区中，缓冲区假定缓冲区足够大，足以
** 					 容纳那么多的样本。此函数只返回当前可用的样本，如果它正在执行过程中，
** 					 可能不是整个样本序列。 
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
int32_t
ADCSequenceDataGet(uint32_t ui32Base, uint32_t ui32SequenceNum,
                   uint32_t *pui32Buffer)
{
    uint32_t ui32Count;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Get the offset of the sequence to be read.
    //
    ui32Base += ADC_SEQ + (ADC_SEQ_STEP * ui32SequenceNum);

    //
    // Read samples from the FIFO until it is empty.
    //
    ui32Count = 0;
    while(!(HWREG(ui32Base + ADC_SSFSTAT) & ADC_SSFSTAT0_EMPTY) &&
          (ui32Count < 8))
    {
        //
        // Read the FIFO and copy it to the destination.
        //
        *pui32Buffer++ = HWREG(ui32Base + ADC_SSFIFO);

        //
        // Increment the count of samples read.
        //
        ui32Count++;
    }

    //
    // Return the number of samples read.
    //
    return(ui32Count);
}

/*******************************************************************************
** 函数名称: ADCProcessorTrigger
** 功能描述: 程序触发端口采样
** 参数说明: ui32Base是 ADC 模块的基址。
**			 ui32SequenceNum 是样本序列号，可以选择包含 ADC_TRIGGER_WAIT 或 ADC_TRIGGER_SIGNAL。
** 返回说明: None
** 描    述: 如果示例序列触发器配置为 ADC_TRIGGER_PROCESSOR（标注：位于 ADCSequenceConfigure()
**           函数的参数 ui32trigger），则此函数将触发处理器启动的样本序列(标注：意思就是会开始启动采样)。
**           如果 ADC_GINGER_WAIT 被固定到序列号中，处理器启动的触发器将被延迟，直到以后由处理器
**           启动的触发器被延迟到指定 ADC_TRIGGER_SIGNAL 的不同 ADC 模块，  
** 创建人员: CLTian
** 创建日期: 2019-07-22
**------------------------------------------------------------------------------
********************************************************************************/
void
ADCProcessorTrigger(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Generate a processor trigger for this sample sequence.
    //
    HWREG(ui32Base + ADC_O_PSSI) |= ((ui32SequenceNum & 0xffff0000) |
                                     (1 << (ui32SequenceNum & 0xf)));
}

//*****************************************************************************
//
//! Configures the software oversampling factor of the ADC.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//! \param ui32Factor is the number of samples to be averaged.
//!
//! This function configures the software oversampling for the ADC, which can
//! be used to provide better resolution on the sampled data.  Oversampling is
//! accomplished by averaging multiple samples from the same analog input.
//! Three different oversampling rates are supported; 2x, 4x, and 8x.
//!
//! Oversampling is only supported on the sample sequencers that are more than
//! one sample in depth (that is, the fourth sample sequencer is not
//! supported).  Oversampling by 2x (for example) divides the depth of the
//! sample sequencer by two; so 2x oversampling on the first sample sequencer
//! can only provide four samples per trigger.  This also means that 8x
//! oversampling is only available on the first sample sequencer.
//!
//! \return None.
//
//*****************************************************************************
void
ADCSoftwareOversampleConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum,
                               uint32_t ui32Factor)
{
    uint32_t ui32Value;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 3);
    ASSERT(((ui32Factor == 2) || (ui32Factor == 4) || (ui32Factor == 8)) &&
           ((ui32SequenceNum == 0) || (ui32Factor != 8)));

    //
    // Convert the oversampling factor to a shift factor.
    //
    for(ui32Value = 0, ui32Factor >>= 1; ui32Factor;
        ui32Value++, ui32Factor >>= 1)
    {
    }

    //
    // Save the shift factor.
    //
    g_pui8OversampleFactor[ui32SequenceNum] = ui32Value;
}

//*****************************************************************************
//
//! Configures a step of the software oversampled sequencer.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//! \param ui32Step is the step to be configured.
//! \param ui32Config is the configuration of this step.
//!
//! This function configures a step of the sample sequencer when using the
//! software oversampling feature.  The number of steps available depends on
//! the oversampling factor set by ADCSoftwareOversampleConfigure().  The value
//! of \e ui32Config is the same as defined for ADCSequenceStepConfigure().
//!
//! \return None.
//
//*****************************************************************************
void
ADCSoftwareOversampleStepConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum,
                                   uint32_t ui32Step, uint32_t ui32Config)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 3);
    ASSERT(((ui32SequenceNum == 0) &&
            (ui32Step < (8 >> g_pui8OversampleFactor[ui32SequenceNum]))) ||
           (ui32Step < (4 >> g_pui8OversampleFactor[ui32SequenceNum])));

    //
    // Get the offset of the sequence to be configured.
    //
    ui32Base += ADC_SEQ + (ADC_SEQ_STEP * ui32SequenceNum);

    //
    // Compute the shift for the bits that control this step.
    //
    ui32Step *= 4 << g_pui8OversampleFactor[ui32SequenceNum];

    //
    // Loop through the hardware steps that make up this step of the software
    // oversampled sequence.
    //
    for(ui32SequenceNum = 1 << g_pui8OversampleFactor[ui32SequenceNum];
        ui32SequenceNum; ui32SequenceNum--)
    {
        //
        // Set the analog mux value for this step.
        //
        HWREG(ui32Base + ADC_SSMUX) = ((HWREG(ui32Base + ADC_SSMUX) &
                                        ~(0x0000000f << ui32Step)) |
                                       ((ui32Config & 0x0f) << ui32Step));

        //
        // Set the upper bits of the analog mux value for this step.
        //
        HWREG(ui32Base + ADC_SSEMUX) = ((HWREG(ui32Base + ADC_SSEMUX) &
                                         ~(0x0000000f << ui32Step)) |
                                        (((ui32Config & 0xf00) >> 8) <<
                                         ui32Step));

        //
        // Set the control value for this step.
        //
        HWREG(ui32Base + ADC_SSCTL) = ((HWREG(ui32Base + ADC_SSCTL) &
                                        ~(0x0000000f << ui32Step)) |
                                       (((ui32Config & 0xf0) >> 4) <<
                                        ui32Step));
        if(ui32SequenceNum != 1)
        {
            HWREG(ui32Base + ADC_SSCTL) &= ~((ADC_SSCTL0_IE0 |
                                              ADC_SSCTL0_END0) << ui32Step);
        }

        //
        // Go to the next hardware step.
        //
        ui32Step += 4;
    }
}

//*****************************************************************************
//
//! Gets the captured data for a sample sequence using software oversampling.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//! \param pui32Buffer is the address where the data is stored.
//! \param ui32Count is the number of samples to be read.
//!
//! This function copies data from the specified sample sequence output FIFO to
//! a memory resident buffer with software oversampling applied.  The requested
//! number of samples are copied into the data buffer; if there are not enough
//! samples in the hardware FIFO to satisfy this many oversampled data items,
//! then incorrect results are returned.  It is the caller's responsibility to
//! read only the samples that are available and wait until enough data is
//! available, for example as a result of receiving an interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
ADCSoftwareOversampleDataGet(uint32_t ui32Base, uint32_t ui32SequenceNum,
                             uint32_t *pui32Buffer, uint32_t ui32Count)
{
    uint32_t ui32Idx, ui32Accum;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 3);
    ASSERT(((ui32SequenceNum == 0) &&
            (ui32Count < (8 >> g_pui8OversampleFactor[ui32SequenceNum]))) ||
           (ui32Count < (4 >> g_pui8OversampleFactor[ui32SequenceNum])));

    //
    // Get the offset of the sequence to be read.
    //
    ui32Base += ADC_SEQ + (ADC_SEQ_STEP * ui32SequenceNum);

    //
    // Read the samples from the FIFO until it is empty.
    //
    while(ui32Count--)
    {
        //
        // Compute the sum of the samples.
        //
        ui32Accum = 0;
        for(ui32Idx = 1 << g_pui8OversampleFactor[ui32SequenceNum]; ui32Idx;
            ui32Idx--)
        {
            //
            // Read the FIFO and add it to the accumulator.
            //
            ui32Accum += HWREG(ui32Base + ADC_SSFIFO);
        }

        //
        // Write the averaged sample to the output buffer.
        //
        *pui32Buffer++ = ui32Accum >> g_pui8OversampleFactor[ui32SequenceNum];
    }
}

//*****************************************************************************
//
//! Configures the hardware oversampling factor of the ADC.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Factor is the number of samples to be averaged.
//!
//! This function configures the hardware oversampling for the ADC, which can
//! be used to provide better resolution on the sampled data.  Oversampling is
//! accomplished by averaging multiple samples from the same analog input.  Six
//! different oversampling rates are supported; 2x, 4x, 8x, 16x, 32x, and 64x.
//! Specifying an oversampling factor of zero disables hardware
//! oversampling.
//!
//! Hardware oversampling applies uniformly to all sample sequencers.  It does
//! not reduce the depth of the sample sequencers like the software
//! oversampling APIs; each sample written into the sample sequencer FIFO is a
//! fully oversampled analog input reading.
//!
//! Enabling hardware averaging increases the precision of the ADC at the cost
//! of throughput.  For example, enabling 4x oversampling reduces the
//! throughput of a 250 k samples/second ADC to 62.5 k samples/second.
//!
//! \return None.
//
//*****************************************************************************
void
ADCHardwareOversampleConfigure(uint32_t ui32Base, uint32_t ui32Factor)
{
    uint32_t ui32Value;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(((ui32Factor == 0) || (ui32Factor == 2) || (ui32Factor == 4) ||
            (ui32Factor == 8) || (ui32Factor == 16) || (ui32Factor == 32) ||
            (ui32Factor == 64)));

    //
    // Convert the oversampling factor to a shift factor.
    //
    for(ui32Value = 0, ui32Factor >>= 1; ui32Factor;
        ui32Value++, ui32Factor >>= 1)
    {
    }

    //
    // Write the shift factor to the ADC to configure the hardware oversampler.
    //
    HWREG(ui32Base + ADC_O_SAC) = ui32Value;
}

//*****************************************************************************
//
//! Configures an ADC digital comparator.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Comp is the index of the comparator to configure.
//! \param ui32Config is the configuration of the comparator.
//!
//! This function configures a comparator.  The \e ui32Config parameter is
//! the result of a logical OR operation between the \b ADC_COMP_TRIG_xxx, and
//! \b ADC_COMP_INT_xxx values.
//!
//! The \b ADC_COMP_TRIG_xxx term can take on the following values:
//!
//! - \b ADC_COMP_TRIG_NONE to never trigger PWM fault condition.
//! - \b ADC_COMP_TRIG_LOW_ALWAYS to always trigger PWM fault condition when
//! ADC output is in the low-band.
//! - \b ADC_COMP_TRIG_LOW_ONCE to trigger PWM fault condition once when ADC
//! output transitions into the low-band.
//! - \b ADC_COMP_TRIG_LOW_HALWAYS to always trigger PWM fault condition when
//! ADC output is in the low-band only if ADC output has been in the high-band
//! since the last trigger output.
//! - \b ADC_COMP_TRIG_LOW_HONCE to trigger PWM fault condition once when ADC
//! output transitions into low-band only if ADC output has been in the
//! high-band since the last trigger output.
//! - \b ADC_COMP_TRIG_MID_ALWAYS to always trigger PWM fault condition when
//! ADC output is in the mid-band.
//! - \b ADC_COMP_TRIG_MID_ONCE to trigger PWM fault condition once when ADC
//! output transitions into the mid-band.
//! - \b ADC_COMP_TRIG_HIGH_ALWAYS to always trigger PWM fault condition when
//! ADC output is in the high-band.
//! - \b ADC_COMP_TRIG_HIGH_ONCE to trigger PWM fault condition once when ADC
//! output transitions into the high-band.
//! - \b ADC_COMP_TRIG_HIGH_HALWAYS to always trigger PWM fault condition when
//! ADC output is in the high-band only if ADC output has been in the low-band
//! since the last trigger output.
//! - \b ADC_COMP_TRIG_HIGH_HONCE to trigger PWM fault condition once when ADC
//! output transitions into high-band only if ADC output has been in the
//! low-band since the last trigger output.
//!
//! The \b ADC_COMP_INT_xxx term can take on the following values:
//!
//! - \b ADC_COMP_INT_NONE to never generate ADC interrupt.
//! - \b ADC_COMP_INT_LOW_ALWAYS to always generate ADC interrupt when ADC
//! output is in the low-band.
//! - \b ADC_COMP_INT_LOW_ONCE to generate ADC interrupt once when ADC output
//! transitions into the low-band.
//! - \b ADC_COMP_INT_LOW_HALWAYS to always generate ADC interrupt when ADC
//! output is in the low-band only if ADC output has been in the high-band
//! since the last trigger output.
//! - \b ADC_COMP_INT_LOW_HONCE to generate ADC interrupt once when ADC output
//! transitions into low-band only if ADC output has been in the high-band
//! since the last trigger output.
//! - \b ADC_COMP_INT_MID_ALWAYS to always generate ADC interrupt when ADC
//! output is in the mid-band.
//! - \b ADC_COMP_INT_MID_ONCE to generate ADC interrupt once when ADC output
//! transitions into the mid-band.
//! - \b ADC_COMP_INT_HIGH_ALWAYS to always generate ADC interrupt when ADC
//! output is in the high-band.
//! - \b ADC_COMP_INT_HIGH_ONCE to generate ADC interrupt once when ADC output
//! transitions into the high-band.
//! - \b ADC_COMP_INT_HIGH_HALWAYS to always generate ADC interrupt when ADC
//! output is in the high-band only if ADC output has been in the low-band
//! since the last trigger output.
//! - \b ADC_COMP_INT_HIGH_HONCE to generate ADC interrupt once when ADC output
//! transitions into high-band only if ADC output has been in the low-band
//! since the last trigger output.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorConfigure(uint32_t ui32Base, uint32_t ui32Comp,
                       uint32_t ui32Config)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32Comp < 8);

    //
    // Save the new setting.
    //
    HWREG(ui32Base + ADC_O_DCCTL0 + (ui32Comp * 4)) = ui32Config;
}

//*****************************************************************************
//
//! Defines the ADC digital comparator regions.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Comp is the index of the comparator to configure.
//! \param ui32LowRef is the reference point for the low/mid band threshold.
//! \param ui32HighRef is the reference point for the mid/high band threshold.
//!
//! The ADC digital comparator operation is based on three ADC value regions:
//! - \b low-band is defined as any ADC value less than or equal to the
//!   \e ui32LowRef value.
//! - \b mid-band is defined as any ADC value greater than the \e ui32LowRef
//!   value but less than or equal to the \e ui32HighRef value.
//! - \b high-band is defined as any ADC value greater than the \e ui32HighRef
//!   value.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorRegionSet(uint32_t ui32Base, uint32_t ui32Comp,
                       uint32_t ui32LowRef, uint32_t ui32HighRef)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32Comp < 8);
    ASSERT((ui32LowRef < 1024) && (ui32LowRef <= ui32HighRef));
    ASSERT(ui32HighRef < 1024);

    //
    // Save the new region settings.
    //
    HWREG(ui32Base + ADC_O_DCCMP0 + (ui32Comp * 4)) = ((ui32HighRef << 16) |
                                                       ui32LowRef);
}

//*****************************************************************************
//
//! Resets the current ADC digital comparator conditions.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Comp is the index of the comparator.
//! \param bTrigger is the flag to indicate reset of Trigger conditions.
//! \param bInterrupt is the flag to indicate reset of Interrupt conditions.
//!
//! Because the digital comparator uses current and previous ADC values, this
//! function allows the comparator to be reset to its initial
//! value to prevent stale data from being used when a sequence is enabled.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorReset(uint32_t ui32Base, uint32_t ui32Comp, bool bTrigger,
                   bool bInterrupt)
{
    uint32_t ui32Temp;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32Comp < 8);

    //
    // Set the appropriate bits to reset the trigger and/or interrupt
    // comparator conditions.
    //
    ui32Temp = 0;
    if(bTrigger)
    {
        ui32Temp |= (1 << (16 + ui32Comp));
    }
    if(bInterrupt)
    {
        ui32Temp |= (1 << ui32Comp);
    }

    HWREG(ui32Base + ADC_O_DCRIC) = ui32Temp;
}

//*****************************************************************************
//
//! Disables a sample sequence comparator interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function disables the requested sample sequence comparator interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorIntDisable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Disable this sample sequence comparator interrupt.
    //
    HWREG(ui32Base + ADC_O_IM) &= ~(0x10000 << ui32SequenceNum);
}

//*****************************************************************************
//
//! Enables a sample sequence comparator interrupt.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! This function enables the requested sample sequence comparator interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorIntEnable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Enable this sample sequence interrupt.
    //
    HWREG(ui32Base + ADC_O_IM) |= 0x10000 << ui32SequenceNum;
}

//*****************************************************************************
//
//! Gets the current comparator interrupt status.
//!
//! \param ui32Base is the base address of the ADC module.
//!
//! This function returns the digital comparator interrupt status bits.  This
//! status is sequence agnostic.
//!
//! \return The current comparator interrupt status.
//
//*****************************************************************************
uint32_t
ADCComparatorIntStatus(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Return the digital comparator interrupt status.
    //
    return(HWREG(ui32Base + ADC_O_DCISC));
}

//*****************************************************************************
//
//! Clears sample sequence comparator interrupt source.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Status is the bit-mapped interrupts status to clear.
//!
//! The specified interrupt status is cleared.
//!
//! \return None.
//
//*****************************************************************************
void
ADCComparatorIntClear(uint32_t ui32Base, uint32_t ui32Status)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Clear the interrupt.
    //
    HWREG(ui32Base + ADC_O_DCISC) = ui32Status;
}

//*****************************************************************************
//
//! Disables ADC interrupt sources.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32IntFlags is the bit mask of the interrupt sources to disable.
//!
//! This function disables the indicated ADC interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! The \e ui32IntFlags parameter is the logical OR of any of the following:
//!
//! - \b ADC_INT_SS0 - interrupt due to ADC sample sequence 0.
//! - \b ADC_INT_SS1 - interrupt due to ADC sample sequence 1.
//! - \b ADC_INT_SS2 - interrupt due to ADC sample sequence 2.
//! - \b ADC_INT_SS3 - interrupt due to ADC sample sequence 3.
//! - \b ADC_INT_DMA_SS0 - interrupt due to DMA on ADC sample sequence 0.
//! - \b ADC_INT_DMA_SS1 - interrupt due to DMA on ADC sample sequence 1.
//! - \b ADC_INT_DMA_SS2 - interrupt due to DMA on ADC sample sequence 2.
//! - \b ADC_INT_DMA_SS3 - interrupt due to DMA on ADC sample sequence 3.
//! - \b ADC_INT_DCON_SS0 - interrupt due to digital comparator on ADC sample
//!   sequence 0.
//! - \b ADC_INT_DCON_SS1 - interrupt due to digital comparator on ADC sample
//!   sequence 1.
//! - \b ADC_INT_DCON_SS2 - interrupt due to digital comparator on ADC sample
//!   sequence 2.
//! - \b ADC_INT_DCON_SS3 - interrupt due to digital comparator on ADC sample
//!   sequence 3.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntDisableEx(uint32_t ui32Base, uint32_t ui32IntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Disable the requested interrupts.
    //
    HWREG(ui32Base + ADC_O_IM) &= ~ui32IntFlags;
}

//*****************************************************************************
//
//! Enables ADC interrupt sources.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32IntFlags is the bit mask of the interrupt sources to disable.
//!
//! This function enables the indicated ADC interrupt sources.  Only the
//! sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! The \e ui32IntFlags parameter is the logical OR of any of the following:
//!
//! - \b ADC_INT_SS0 - interrupt due to ADC sample sequence 0.
//! - \b ADC_INT_SS1 - interrupt due to ADC sample sequence 1.
//! - \b ADC_INT_SS2 - interrupt due to ADC sample sequence 2.
//! - \b ADC_INT_SS3 - interrupt due to ADC sample sequence 3.
//! - \b ADC_INT_DMA_SS0 - interrupt due to DMA on ADC sample sequence 0.
//! - \b ADC_INT_DMA_SS1 - interrupt due to DMA on ADC sample sequence 1.
//! - \b ADC_INT_DMA_SS2 - interrupt due to DMA on ADC sample sequence 2.
//! - \b ADC_INT_DMA_SS3 - interrupt due to DMA on ADC sample sequence 3.
//! - \b ADC_INT_DCON_SS0 - interrupt due to digital comparator on ADC sample
//!   sequence 0.
//! - \b ADC_INT_DCON_SS1 - interrupt due to digital comparator on ADC sample
//!   sequence 1.
//! - \b ADC_INT_DCON_SS2 - interrupt due to digital comparator on ADC sample
//!   sequence 2.
//! - \b ADC_INT_DCON_SS3 - interrupt due to digital comparator on ADC sample
//!   sequence 3.
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntEnableEx(uint32_t ui32Base, uint32_t ui32IntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Enable the requested interrupts.
    //
    HWREG(ui32Base + ADC_O_IM) |= ui32IntFlags;
}

//*****************************************************************************
//
//! Gets interrupt status for the specified ADC module.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param bMasked specifies whether masked or raw interrupt status is
//! returned.
//!
//! If \e bMasked is set as \b true, then the masked interrupt status is
//! returned; otherwise, the raw interrupt status is returned.
//!
//! \return Returns the current interrupt status for the specified ADC module.
//! The value returned is the logical OR of the \b ADC_INT_* values that are
//! currently active.
//
//*****************************************************************************
uint32_t
ADCIntStatusEx(uint32_t ui32Base, bool bMasked)
{
    uint32_t ui32Temp;

    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Return either the masked interrupt status or the raw interrupt status as
    // requested.
    //
    if(bMasked)
    {
        ui32Temp = HWREG(ui32Base + ADC_O_ISC);
    }
    else
    {
        //
        // Read the Raw interrupt status to see if a digital comparator
        // interrupt is active.
        //
        ui32Temp = HWREG(ui32Base + ADC_O_RIS);

        //
        // Since, the raw interrupt status only indicates that any one of the
        // digital comparators caused an interrupt, if the raw interrupt status
        // is set then the return value is modified to indicate that all sample
        // sequences have a pending digital comparator interrupt.
        // This is exactly how the hardware works so the return code is
        // modified to match this behavior.
        //
        if(ui32Temp & ADC_RIS_INRDC)
        {
            ui32Temp |= (ADC_INT_DCON_SS3 | ADC_INT_DCON_SS2 |
                         ADC_INT_DCON_SS1 | ADC_INT_DCON_SS0);
        }
    }
    return(ui32Temp);
}

//*****************************************************************************
//
//! Clears the specified ADC interrupt sources.
//!
//! \param ui32Base is the base address of the ADC port.
//! \param ui32IntFlags is the bit mask of the interrupt sources to disable.
//!
//! Clears the interrupt for the specified interrupt source(s).
//!
//! The \e ui32IntFlags parameter is the logical OR of the \b ADC_INT_* values.
//! See the ADCIntEnableEx() function for the list of possible \b ADC_INT*
//! values.
//!
//! \note Because there is a write buffer in the Cortex-M processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
void
ADCIntClearEx(uint32_t ui32Base, uint32_t ui32IntFlags)
{
    HWREG(ui32Base + ADC_O_ISC) |= ui32IntFlags;
}

//*****************************************************************************
//
//! Selects the ADC reference.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Ref is the reference to use.
//!
//! The ADC reference is set as specified by \e ui32Ref.  It must be one of
//! \b ADC_REF_INT, \b ADC_REF_EXT_3V, or \b ADC_REF_EXT_1V for internal or
//! external reference.  If \b ADC_REF_INT is chosen, then an internal 3V
//! reference is used and no external reference is needed.  If
//! \b ADC_REF_EXT_3V is chosen, then a 3V reference must be supplied to the
//! AVREF pin.  If \b ADC_REF_EXT_1V is chosen, then a 1V external reference
//! must be supplied to the AVREF pin.
//!
//! \note The ADC reference can only be selected on parts that have an external
//! reference.  Consult the data sheet for your part to determine if there is
//! an external reference.
//!
//! \return None.
//
//*****************************************************************************
void
ADCReferenceSet(uint32_t ui32Base, uint32_t ui32Ref)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT((ui32Ref == ADC_REF_INT) || (ui32Ref == ADC_REF_EXT_3V) ||
           (ui32Ref == ADC_REF_EXT_1V));

    //
    // Set the reference.
    //
    HWREG(ui32Base + ADC_O_CTL) =
        (HWREG(ui32Base + ADC_O_CTL) & ~ADC_CTL_VREF_M) | ui32Ref;
}

//*****************************************************************************
//
//! Returns the current setting of the ADC reference.
//!
//! \param ui32Base is the base address of the ADC module.
//!
//! Returns the value of the ADC reference setting.  The returned value is one
//! of \b ADC_REF_INT, \b ADC_REF_EXT_3V, or \b ADC_REF_EXT_1V.
//!
//! \note The value returned by this function is only meaningful if used on a
//! part that is capable of using an external reference.  Consult the data
//! sheet for your part to determine if it has an external reference input.
//!
//! \return The current setting of the ADC reference.
//
//*****************************************************************************
uint32_t
ADCReferenceGet(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Return the value of the reference.
    //
    return(HWREG(ui32Base + ADC_O_CTL) & ADC_CTL_VREF_M);
}

//*****************************************************************************
//
//! Sets the phase delay between a trigger and the start of a sequence.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32Phase is the phase delay, specified as one of \b ADC_PHASE_0,
//! \b ADC_PHASE_22_5, \b ADC_PHASE_45, \b ADC_PHASE_67_5, \b ADC_PHASE_90,
//! \b ADC_PHASE_112_5, \b ADC_PHASE_135, \b ADC_PHASE_157_5, \b ADC_PHASE_180,
//! \b ADC_PHASE_202_5, \b ADC_PHASE_225, \b ADC_PHASE_247_5, \b ADC_PHASE_270,
//! \b ADC_PHASE_292_5, \b ADC_PHASE_315, or \b ADC_PHASE_337_5.
//!
//! This function sets the phase delay between the detection of an ADC trigger
//! event and the start of the sample sequence.  By selecting a different phase
//! delay for a pair of ADC modules (such as \b ADC_PHASE_0 and
//! \b ADC_PHASE_180) and having each ADC module sample the same analog input,
//! it is possible to increase the sampling rate of the analog input (with
//! samples N, N+2, N+4, and so on, coming from the first ADC and samples N+1,
//! N+3, N+5, and so on, coming from the second ADC).  The ADC module has a
//! single phase delay that is applied to all sample sequences within that
//! module.
//!
//! \note This capability is not available on all parts.
//!
//! \return None.
//
//*****************************************************************************
void
ADCPhaseDelaySet(uint32_t ui32Base, uint32_t ui32Phase)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT((ui32Phase == ADC_PHASE_0) || (ui32Phase == ADC_PHASE_22_5) ||
           (ui32Phase == ADC_PHASE_45) || (ui32Phase == ADC_PHASE_67_5) ||
           (ui32Phase == ADC_PHASE_90) || (ui32Phase == ADC_PHASE_112_5) ||
           (ui32Phase == ADC_PHASE_135) || (ui32Phase == ADC_PHASE_157_5) ||
           (ui32Phase == ADC_PHASE_180) || (ui32Phase == ADC_PHASE_202_5) ||
           (ui32Phase == ADC_PHASE_225) || (ui32Phase == ADC_PHASE_247_5) ||
           (ui32Phase == ADC_PHASE_270) || (ui32Phase == ADC_PHASE_292_5) ||
           (ui32Phase == ADC_PHASE_315) || (ui32Phase == ADC_PHASE_337_5));

    //
    // Set the phase delay.
    //
    HWREG(ui32Base + ADC_O_SPC) = ui32Phase;
}

//*****************************************************************************
//
//! Gets the phase delay between a trigger and the start of a sequence.
//!
//! \param ui32Base is the base address of the ADC module.
//!
//! This function gets the current phase delay between the detection of an ADC
//! trigger event and the start of the sample sequence.
//!
//! \return Returns the phase delay, specified as one of \b ADC_PHASE_0,
//! \b ADC_PHASE_22_5, \b ADC_PHASE_45, \b ADC_PHASE_67_5, \b ADC_PHASE_90,
//! \b ADC_PHASE_112_5, \b ADC_PHASE_135, \b ADC_PHASE_157_5, \b ADC_PHASE_180,
//! \b ADC_PHASE_202_5, \b ADC_PHASE_225, \b ADC_PHASE_247_5, \b ADC_PHASE_270,
//! \b ADC_PHASE_292_5, \b ADC_PHASE_315, or \b ADC_PHASE_337_5.
//
//*****************************************************************************
uint32_t
ADCPhaseDelayGet(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Return the phase delay.
    //
    return(HWREG(ui32Base + ADC_O_SPC));
}

//*****************************************************************************
//
//! Enables DMA for sample sequencers.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! Allows DMA requests to be generated based on the FIFO level of the sample
//! sequencer.
//!
//! \return None.
//
//*****************************************************************************
void
ADCSequenceDMAEnable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Enable the DMA on the specified sequencer.
    //
    HWREG(ui32Base + ADC_O_ACTSS) |= 0x100 << ui32SequenceNum;
}

//*****************************************************************************
//
//! Disables DMA for sample sequencers.
//!
//! \param ui32Base is the base address of the ADC module.
//! \param ui32SequenceNum is the sample sequence number.
//!
//! Prevents the specified sample sequencer from generating DMA requests.
//!
//! \return None.
//
//*****************************************************************************
void
ADCSequenceDMADisable(uint32_t ui32Base, uint32_t ui32SequenceNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));
    ASSERT(ui32SequenceNum < 4);

    //
    // Disable the DMA on the specified sequencer.
    //
    HWREG(ui32Base + ADC_O_ACTSS) &= ~(0x100 << ui32SequenceNum);
}

//*****************************************************************************
//
//! Determines whether the ADC is busy or not.
//!
//! \param ui32Base is the base address of the ADC.
//!
//! This function allows the caller to determine whether or not the ADC is
//! currently sampling .  If \b false is returned, then the ADC is not
//! sampling data.
//!
//! Use this function to detect that the ADC is finished sampling data before
//! putting the device into deep sleep.  Before using this function, it is
//! highly recommended that the event trigger is changed to
//! \b ADC_TRIGGER_NEVER on all enabled sequencers to prevent the ADC from
//! starting after checking the busy status.
//!
//! \return Returns \b true if the ADC is sampling or \b false if all
//! samples are complete.
//
//*****************************************************************************
bool
ADCBusy(uint32_t ui32Base)
{
    //
    // Check the argument.
    //
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    //
    // Determine if the ADC is busy.
    //
    return((HWREG(ui32Base + ADC_O_ACTSS) & ADC_ACTSS_BUSY) ? true : false);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
