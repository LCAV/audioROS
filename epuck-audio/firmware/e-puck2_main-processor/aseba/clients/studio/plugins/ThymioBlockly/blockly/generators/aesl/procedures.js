/**
 * @fileoverview Generating AESL for procedure blocks.
 * @author Fabian Hahn (fabian@hahn.graphics)
 */
'use strict';

goog.provide('Blockly.AESL.procedures');

goog.require('Blockly.AESL');

Blockly.AESL['procedures_defnoreturn'] = function(block)
{
	// Define a procedure with a return value.
	var funcName = Blockly.AESL.variableDB_.getName(block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
	var branch = Blockly.AESL.statementToCode(block, 'STACK');
	
	if(branch.indexOf('\tcallsub ') != -1) {
		// AESL does allow subroutines to call other subroutines, but they need to be defined in the correct
		// order. Since we currently no way to track this in Blockly, let's just disallow calling other subs
		// altogether
		Blockly.AESL.addSubroutine(funcName, '');
		return null;
	}

	// Ignore input arguments - AESL subroutines cannot receive or return values
	
	Blockly.AESL.addSubroutine(funcName, branch);
	return null;
};

Blockly.AESL['procedures_callnoreturn'] = function(block)
{
	// Call a procedure with no return value.
	var funcName = Blockly.AESL.variableDB_.getName(block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
	return 'callsub ' + funcName + '\n';
};