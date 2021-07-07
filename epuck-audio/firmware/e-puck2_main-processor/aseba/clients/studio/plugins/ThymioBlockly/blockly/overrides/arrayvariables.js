/**
 * @fileoverview Override for array variables
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

/**
 * Construct the blocks required by the flyout for the variable category.
 * 
 * @param {!Array.
 *            <!Blockly.Block>} blocks List of blocks to show.
 * @param {!Array.
 *            <number>} gaps List of widths between blocks.
 * @param {number}
 *            margin Standard margin width for calculating gaps.
 * @param {!Blockly.Workspace}
 *            workspace The flyout's workspace.
 */
Blockly.Variables.flyoutCategory = function(blocks, gaps, margin, workspace)
{
	if(Blockly.Blocks['thymio_declare_array']) {
		var block = Blockly.Block.obtain(workspace, 'thymio_declare_array');
		block.initSvg();
		blocks.push(block);
		gaps.push(margin * 2);
	}

	var variableList = Blockly.Variables.allVariables(workspace.targetWorkspace);
	variableList.sort(goog.string.caseInsensitiveCompare);
	// In addition to the user's variables, we also want to display the default
	// variable name at the top. We also don't want this duplicated if the
	// user has created a variable of the same name.
	variableList.unshift(null);
	var defaultVariable = undefined;
	for(var i = 0; i < variableList.length; i++) {
		if(variableList[i] === defaultVariable) {
			continue;
		}
		var getBlock = Blockly.Blocks['thymio_variable_get'] ? Blockly.Block.obtain(workspace, 'thymio_variable_get') : null;
		getBlock && getBlock.initSvg();
		var setBlock = Blockly.Blocks['thymio_variable_set'] ? Blockly.Block.obtain(workspace, 'thymio_variable_set') : null;
		setBlock && setBlock.initSvg();
		var getArrayBlock = Blockly.Blocks['thymio_get_array'] ? Blockly.Block.obtain(workspace, 'thymio_get_array') : null;
		getArrayBlock && getArrayBlock.initSvg();
		var setArrayBlock = Blockly.Blocks['thymio_set_array'] ? Blockly.Block.obtain(workspace, 'thymio_set_array') : null;
		setArrayBlock && setArrayBlock.initSvg();
		
		var setValueBlock = Blockly.Blocks['math_number'] ? Blockly.Block.obtain(workspace, 'math_number') : null;
		if(setValueBlock) {
			setValueBlock.setShadow(true);
			setValueBlock.initSvg();
			setBlock.inputList[0].connection.connect(setValueBlock.outputConnection);
			setValueBlock.render();
		}
		
		var getArrayIndexBlock = Blockly.Blocks['math_number'] ? Blockly.Block.obtain(workspace, 'math_number') : null;
		if(getArrayIndexBlock) {
			getArrayIndexBlock.setShadow(true);
			getArrayIndexBlock.initSvg();
			getArrayBlock.inputList[0].connection.connect(getArrayIndexBlock.outputConnection);
			getArrayIndexBlock.setFieldValue('0', 'NUM');
			getArrayIndexBlock.render();
		}		
		
		var setArrayIndexBlock = Blockly.Blocks['math_number'] ? Blockly.Block.obtain(workspace, 'math_number') : null;
		if(setArrayIndexBlock) {
			setArrayIndexBlock.setShadow(true);
			setArrayIndexBlock.initSvg();
			setArrayBlock.inputList[0].connection.connect(setArrayIndexBlock.outputConnection);
			setArrayIndexBlock.setFieldValue('0', 'NUM');
			setArrayIndexBlock.render();
		}
		
		var setArrayValueBlock = Blockly.Blocks['math_number'] ? Blockly.Block.obtain(workspace, 'math_number') : null;
		if(setArrayValueBlock) {
			setArrayValueBlock.setShadow(true);
			setArrayValueBlock.initSvg();
			setArrayBlock.inputList[1].connection.connect(setArrayValueBlock.outputConnection);
			setArrayValueBlock.render();
		}

		if(variableList[i] === null) {
			defaultVariable = (getBlock || setBlock || getArrayBlock || setArrayBlock).getVars()[0];
		} else {
			getBlock && getBlock.setFieldValue(variableList[i], 'VAR');
			setBlock && setBlock.setFieldValue(variableList[i], 'VAR');
			getArrayBlock && getArrayBlock.setFieldValue(variableList[i], 'VAR');
			setArrayBlock && setArrayBlock.setFieldValue(variableList[i], 'VAR');
		}
		setBlock && blocks.push(setBlock);
		getBlock && blocks.push(getBlock);
		setArrayBlock && blocks.push(setArrayBlock);
		getArrayBlock && blocks.push(getArrayBlock);
		if(getBlock && setBlock && getArrayBlock && setArrayBlock) {
			gaps.push(margin, margin, margin, margin * 3);
		} else {
			gaps.push(margin * 2);
		}
	}
};
