/**
 * @fileoverview Generating AESL for loops blocks.
 * @author Fabian Hahn (fabian@hahn.graphics)
 */
'use strict';

goog.provide('Blockly.AESL.loops');

goog.require('Blockly.AESL');

Blockly.AESL['controls_repeat'] = function(block)
{
	var repeats = String(Number(block.getFieldValue('TIMES')));

	var branch = Blockly.AESL.statementToCode(block, 'DO');
	
	var loopVar = Blockly.AESL.variableDB_.getDistinctName('i', Blockly.Variables.NAME_TYPE);
	
	var code = 'for ' + loopVar + ' in 1:' + repeats + ' do\n' + branch + 'end\n';
	
	return code;
};

Blockly.AESL['controls_whileUntil'] = function(block)
{
	// Do while/until loop.
	var until = block.getFieldValue('MODE') == 'UNTIL';
	var argument0 = Blockly.AESL.valueToCode(block, 'BOOL', until ? Blockly.AESL.ORDER_LOGICAL_NOT : Blockly.AESL.ORDER_NONE) || '0 == 1';
	var branch = Blockly.AESL.statementToCode(block, 'DO');
	
	if(until) {
		argument0 = 'not ' + argument0;
	}
	
	return 'while ' + argument0 + ' do\n' + branch + 'end\n';
};
