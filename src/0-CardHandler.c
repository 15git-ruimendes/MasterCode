/**
 * @file 0-CardHandler.c
 * @author Rui Mendes 
 * @brief 
 * @version 0.1
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "0-CardHandler.h"

/**
 * CAN Id Builder
 */
uint32_t idBuilder(Master* Card,uint8_t Dest, uint8_t Function)
{
	// Build identifier
	uint32_t Header = Card->Id<<7;
	Header = Header + (Dest << 3);
	Header = Header + (Function);
	Header = Header << 5;

	return Header;
}

/**
 * CAN Id Parser
 */
uint8_t idParser(Master* Card, uint32_t Id)
{
	// Save Rx Id in separate variable
		uint16_t Header = Id;

		// Parse
		uint8_t Source = (Header >> 5) >> 7;
		uint8_t Destination = ((Header >> 5) >> 3) & 0b00000000000000000000000000001111;
		uint8_t Function = (Header >> 5) & 0b00000000000000000000000000000111;

        
		Card->SourceRx = Source;
        Card->FunctionRx = Function;
		// Return Function
		return Function;
}

/**
 * @brief Integration tester for the cards in the system.  
 * @note   Function must be run for each of the systems card and 
 * return values must be registered appropriately.
 * @param  Card: Master card pointer
 * @param  CardId: Slave card id's. Check global definitions
 * @retval None
 */
void CardTester(Master* Card,uint8_t CardId);