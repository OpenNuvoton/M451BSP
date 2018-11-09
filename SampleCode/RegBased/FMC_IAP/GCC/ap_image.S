;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv7-m

	.globl	loaderImageBase
	.globl	loaderImageLimit

	.align	4
 
	.text
        
loaderImageBase:
    .incbin  "..\\..\\LDROM_iap\\LDROM_iap.bin"
loaderImageLimit:
    .space   4
    
    .end
   
