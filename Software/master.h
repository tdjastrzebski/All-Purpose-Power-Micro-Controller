/*---------------------------------------------------------------------------------------------
 *  Copyright (c) 2022 Tomasz Jastrzębski. All rights reserved.
 *  Licensed under the MIT License. See License.txt in the project root for license information.
 *--------------------------------------------------------------------------------------------*/

#ifndef __MASTER_H
#define __MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

// to be included in the main.c "USER CODE 1" section
void PreInit(void);
// to be included in the main.c "USER CODE Init" section
void Init(void);
// to be included in the main.c "USER CODE SysInit" section
void SysInit(void);
// to be included in the main.c "USER CODE 2" section
void PostInit(void);
// to be included in the main.c "USER CODE 3" section
void MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif
