/*******************************************************************************
Copyright (C) 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of STMicroelectronics nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file  VL53LX_platform_log.h
 *
 * @brief EwokPlus25 platform logging function definition
 */


#ifndef _VL53LX_PLATFORM_LOG_H_
#define _VL53LX_PLATFORM_LOG_H_

#include "vl53lx_platform_user_config.h"

#ifdef VL53LX_LOG_ENABLE

	#ifdef _MSC_VER
	#   define EWOKPLUS_EXPORTS  __declspec(dllexport)
	#else
	#       define EWOKPLUS_EXPORTS
	#endif

	#include <vl53l3_types.h>

	#ifdef __cplusplus
	extern "C" {
	#endif

	#include <time.h>

	/**
	 * @brief Set the level, output and specific functions for module logging.
	 *
	 *
	 * @param filename  - full path of output log file, NULL for print to stdout
	 *
	 * @param modules   - Module or None or All to trace
	 *                      VL53LX_TRACE_MODULE_NONE
	 *                      VL53LX_TRACE_MODULE_API
	 *                      VL53LX_TRACE_MODULE_CORE
	 *                      VL53LX_TRACE_MODULE_TUNING
	 *                      VL53LX_TRACE_MODULE_CHARACTERISATION
	 *                      VL53LX_TRACE_MODULE_PLATFORM
	 *                      VL53LX_TRACE_MODULE_ALL
	 *
	 * @param level     - trace level
	 *                      VL53LX_TRACE_LEVEL_NONE
	 *                      VL53LX_TRACE_LEVEL_ERRORS
	 *                      VL53LX_TRACE_LEVEL_WARNING
	 *                      VL53LX_TRACE_LEVEL_INFO
	 *                      VL53LX_TRACE_LEVEL_DEBUG
	 *                      VL53LX_TRACE_LEVEL_ALL
	 *                      VL53LX_TRACE_LEVEL_IGNORE
	 *
	 *  @param functions - function level to trace;
	 *                      VL53LX_TRACE_FUNCTION_NONE
	 *                      VL53LX_TRACE_FUNCTION_I2C
	 *                      VL53LX_TRACE_FUNCTION_ALL
	 *
	 * @return status - always VL53LX_ERROR_NONE
	 *
	 */

	#define		VL53LX_TRACE_LEVEL_NONE			0x00000000
	#define		VL53LX_TRACE_LEVEL_ERRORS		0x00000001
	#define		VL53LX_TRACE_LEVEL_WARNING		0x00000002
	#define		VL53LX_TRACE_LEVEL_INFO			0x00000004
	#define		VL53LX_TRACE_LEVEL_DEBUG		0x00000008
	#define		VL53LX_TRACE_LEVEL_ALL			0x00000010
	#define		VL53LX_TRACE_LEVEL_IGNORE		0x00000020

	#define		VL53LX_TRACE_FUNCTION_NONE		0x00000000
	#define		VL53LX_TRACE_FUNCTION_I2C		0x00000001
	#define		VL53LX_TRACE_FUNCTION_ALL		0x7fffffff

	#define		VL53LX_TRACE_MODULE_NONE		0x00000000
	#define		VL53LX_TRACE_MODULE_API			0x00000001
	#define		VL53LX_TRACE_MODULE_CORE		0x00000002
	#define		VL53LX_TRACE_MODULE_PROTECTED		0x00000004
	#define		VL53LX_TRACE_MODULE_HISTOGRAM		0x00000008
	#define		VL53LX_TRACE_MODULE_REGISTERS		0x00000010
	#define		VL53LX_TRACE_MODULE_PLATFORM		0x00000020
	#define		VL53LX_TRACE_MODULE_NVM			0x00000040
	#define		VL53LX_TRACE_MODULE_CALIBRATION_DATA	0x00000080
	#define		VL53LX_TRACE_MODULE_NVM_DATA		0x00000100
	#define		VL53LX_TRACE_MODULE_HISTOGRAM_DATA	0x00000200
	#define		VL53LX_TRACE_MODULE_RANGE_RESULTS_DATA	0x00000400
	#define		VL53LX_TRACE_MODULE_XTALK_DATA		0x00000800
	#define		VL53LX_TRACE_MODULE_OFFSET_DATA		0x00001000
	#define		VL53LX_TRACE_MODULE_DATA_INIT		0x00002000
    #define		VL53LX_TRACE_MODULE_REF_SPAD_CHAR	0x00004000
    #define		VL53LX_TRACE_MODULE_SPAD_RATE_MAP	0x00008000
	//#ifdef PAL_EXTENDED // changed by PYT to allow spad map
		#define	VL53LX_TRACE_MODULE_SPAD		0x01000000
	#ifdef PAL_EXTENDED
		#define	VL53LX_TRACE_MODULE_FMT			0x02000000
		#define	VL53LX_TRACE_MODULE_UTILS		0x04000000
	#endif
	#define		VL53LX_TRACE_MODULE_CUSTOMER_API	0x40000000
	#define		VL53LX_TRACE_MODULE_ALL			0x7fffffff


	extern uint32_t _trace_level;

	/*
	 * NOTE: dynamically exported if we enable logging.
	 *       this way, Python interfaces can access this function, but we don't
	 *       need to include it in the .def files.
	 */
	EWOKPLUS_EXPORTS int8_t VL53LX_trace_config(
		char *filename,
		uint32_t modules,
		uint32_t level,
		uint32_t functions);

	/**
	 * @brief Print trace module function.
	 *
	 * @param module   - ??
	 * @param level    - ??
	 * @param function - ??
	 * @param format   - ??
	 *
	 */

	EWOKPLUS_EXPORTS void VL53LX_trace_print_module_function(
		uint32_t module,
		uint32_t level,
		uint32_t function,
		const char *format, ...);

	/**
	 * @brief Get global _trace_functions parameter
	 *
	 * @return _trace_functions
	 */

	uint32_t VL53LX_get_trace_functions(void);

	/**
	 * @brief Set global _trace_functions parameter
	 *
	 * @param[in] function : new function code
	 */

	void VL53LX_set_trace_functions(uint32_t function);


	/*
	 * @brief Returns the current system tick count in [ms]
	 *
	 * @return  time_ms : current time in [ms]
	 *
	 */

	uint32_t VL53LX_clock(void);

	#define LOG_GET_TIME() \
		((int)VL53LX_clock())

	#define _LOG_TRACE_PRINT(module, level, function, ...) \
		VL53LX_trace_print_module_function(module, level, function, ##__VA_ARGS__);

	#define _LOG_FUNCTION_START(module, fmt, ...) \
		VL53LX_trace_print_module_function(module, _trace_level, VL53LX_TRACE_FUNCTION_ALL, "%6ld <START> %s "fmt"\n", LOG_GET_TIME(), __FUNCTION__, ##__VA_ARGS__);

	#define	_LOG_FUNCTION_END(module, status, ...)\
		VL53LX_trace_print_module_function(module, _trace_level, VL53LX_TRACE_FUNCTION_ALL, "%6ld <END> %s %d\n", LOG_GET_TIME(), __FUNCTION__, (int)status, ##__VA_ARGS__)

	#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)\
		VL53LX_trace_print_module_function(module, _trace_level, VL53LX_TRACE_FUNCTION_ALL, "%6ld <END> %s %d "fmt"\n", LOG_GET_TIME(),  __FUNCTION__, (int)status, ##__VA_ARGS__)

	#define _LOG_GET_TRACE_FUNCTIONS()\
		VL53LX_get_trace_functions()

	#define _LOG_SET_TRACE_FUNCTIONS(functions)\
		VL53LX_set_trace_functions(functions)

	#define _LOG_STRING_BUFFER(x) char x[VL53LX_MAX_STRING_LENGTH]

	#ifdef __cplusplus
	}
	#endif

#else /* VL53LX_LOG_ENABLE - no logging */

	#define _LOG_TRACE_PRINT(module, level, function, ...)
	#define _LOG_FUNCTION_START(module, fmt, ...)
	#define _LOG_FUNCTION_END(module, status, ...)
	#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)
	#define _LOG_GET_TRACE_FUNCTIONS() 0
	#define _LOG_SET_TRACE_FUNCTIONS(functions)
	#define _LOG_STRING_BUFFER(x)

#endif /* VL53LX_LOG_ENABLE */

#endif  /* _VL53LX_PLATFORM_LOG_H_ */
