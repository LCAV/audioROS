/**
 * @fileoverview Generating AESL for math blocks.
 * @author Fabian Hahn (fabian@hahn.graphics)
 */
'use strict';

goog.provide('Blockly.AESL.math');

goog.require('Blockly.AESL');

Blockly.AESL['math_number'] = function(block)
{
	// Numeric value.
	var code = parseInt(block.getFieldValue('NUM'));
	return [code, Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['math_arithmetic'] = function(block)
{
	// Basic arithmetic operators, and power.
	var OPERATORS = {
		'ADD' : [' + ', Blockly.AESL.ORDER_ADD],
		'MINUS' : [' - ', Blockly.AESL.ORDER_ADD],
		'MULTIPLY' : [' * ', Blockly.AESL.ORDER_MULT],
		'DIVIDE' : [' / ', Blockly.AESL.ORDER_MULT],
		'POWER' : [null, Blockly.AESL.ORDER_MULT]
	// Handle power separately.
	};
	var tuple = OPERATORS[block.getFieldValue('OP')];
	var operator = tuple[0];
	var order = tuple[1];
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order) || '0';
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order) || '0';
	var code;
	
	// AESL cannot do power, just return first argument
	if(!operator) {
		return [argument0, Blockly.AESL.ORDER_MULT];
	}
	
	code = argument0 + operator + argument1;
	return [code, order];
};
