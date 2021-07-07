/**
 * @fileoverview Generating AESL for variable blocks.
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

goog.provide('Blockly.AESL.variables');

goog.require('Blockly.AESL');

Blockly.AESL['variables_get'] = function(block)
{
	// Variable getter.
	var code = '@variable:' + Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE) + '@';
	
	return [code, Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['variables_set'] = function(block)
{
	var varName = Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE);
	
	// AESL only allows int variables, so check if our input has the correct type
	var valueBlock = this.getInputTargetBlock('VALUE');
	
	var argument0;	
    if(!Blockly.AESL.checkValueBlockType(valueBlock)) {
    	argument0 = '0';
    } else {
    	argument0 = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';
    }
	
	// Variable setter.
	return '@variable:' + varName + '@ = ' + argument0 + '\n';
};
