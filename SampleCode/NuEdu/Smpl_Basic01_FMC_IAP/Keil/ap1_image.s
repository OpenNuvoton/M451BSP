;/******************************************************************************
; * @note
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/


    AREA _image, DATA, READONLY

    EXPORT  loaderImage1Base
    EXPORT  loaderImage1Limit
    
    ALIGN   4
        
loaderImage1Base
    INCBIN ./obj/fmc_ld_boot.bin
loaderImage1Limit

    
    END