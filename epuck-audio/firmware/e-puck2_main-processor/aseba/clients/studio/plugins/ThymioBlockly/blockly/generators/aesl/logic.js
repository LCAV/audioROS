/**
 * @fileoverview Generating AESL for logic blocks.
 * @author Fabian Hahn (fabian@hahn.graphics)
 */
'use strict';

goog.provide('Blockly.AESL.logic');

goog.require('Blockly.AESL');

Blockly.AESL['controls_if'] = function(block)
{
	// If/elseif/else condition.
	var n = 0;
	var argument = Blockly.AESL.valueToCode(block, 'IF' + n, Blockly.AESL.ORDER_NONE) || '0 == 1';
	var branch = Blockly.AESL.statementToCode(block, 'DO' + n);

	var code = 'if ' + argument + ' then\n' + branch;

	for(n = 1; n <= block.elseifCount_; n++) {
		argument = Blockly.AESL.valueToCode(block, 'IF' + n, Blockly.AESL.ORDER_NONE) || '0 == 1';
		branch = Blockly.AESL.statementToCode(block, 'DO' + n);
		code += 'elseif ' + argument + ' then\n' + branch;
	}

	if(block.elseCount_) {
		branch = Blockly.AESL.statementToCode(block, 'ELSE');
		code += 'else\n' + branch;
	}

	code += 'end\n';

	return code;
};

Blockly.AESL['logic_compare'] = function(block)
{
	// AESL only allows int comparisons, so check if both arguments have the correct type
	var blockA = this.getInputTargetBlock('A');
    var blockB = this.getInputTargetBlock('B');
	
    if(!Blockly.AESL.checkValueBlockType(blockA) || !Blockly.AESL.checkValueBlockType(blockB)) {
    	return ['0 == 1', Blockly.AESL.ORDER_CONDITION];
    }
    
	// Comparison operator.
	var OPERATORS = {
		'EQ' : '==',
		'NEQ' : '!=',
		'LT' : '<',
		'LTE' : '<=',
		'GT' : '>',
		'GTE' : '>='
	};
	var operator = OPERATORS[block.getFieldValue('OP')];
	var order = Blockly.AESL.ORDER_CONDITION;
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order) || '0';
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order) || '0';
	var code = argument0 + ' ' + operator + ' ' + argument1;
	return [code, order];
};

Blockly.AESL['logic_operation'] = function(block)
{
	// Operations 'and', 'or'.
	var operator = (block.getFieldValue('OP') == 'AND') ? 'and' : 'or';
	var order = (operator == 'and') ? Blockly.AESL.ORDER_LOGICAL_AND : Blockly.AESL.ORDER_LOGICAL_OR;
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order);
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order);
	
	if(!argument0 && !argument1) {
		// If there are no arguments, then the return value is false.
		return ['0 == 1', Blockly.AESL.ORDER_CONDITION];
	} else {
		if(!argument0) {
			return [argument1, order];
		}
		
		if(!argument1) {
			return [argument0, order];
		}
	}
	
	var code = argument0 + ' ' + operator + ' ' + argument1;
	return [code, order];
};

Blockly.AESL['logic_negate'] = function(block)
{
	// Negation.
	var order = Blockly.AESL.ORDER_LOGICAL_NOT;
	var argument0 = Blockly.AESL.valueToCode(block, 'BOOL', order) || '0 == 0';
	var code = 'not ' + argument0;
	return [code, order];
};

Blockly.AESL['logic_boolean'] = function(block)
{
	// Boolean values true and false.
	var code = (block.getFieldValue('BOOL') == 'TRUE') ? '0 == 0' : '0 == 1';
	return [code, Blockly.AESL.ORDER_CONDITION];
};
