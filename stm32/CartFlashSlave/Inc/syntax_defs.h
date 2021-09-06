/*
 * syntax_sugar.h
 *
 *  Created on: 12 sie 2021
 *      Author: alf64
 */

#ifndef SYNTAX_DEFS_H_
#define SYNTAX_DEFS_H_

//!< Some defs that may be useful for register/bit operations.
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#define RESET       0
#define SET         (!RESET)
#define DISABLE     RESET
#define ENABLE      SET
#define SUCCESS     0
#define ERROR       (!SUCCESS)

#endif /* SYNTAX_DEFS_H_ */
