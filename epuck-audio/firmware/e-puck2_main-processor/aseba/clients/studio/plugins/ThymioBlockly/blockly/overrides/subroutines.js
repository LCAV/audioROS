/**
 * @fileoverview Override for subroutines
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

/**
 * Construct the blocks required by the flyout for the procedure category.
 * @param {!Array.<!Blockly.Block>} blocks List of blocks to show.
 * @param {!Array.<number>} gaps List of widths between blocks.
 * @param {number} margin Standard margin width for calculating gaps.
 * @param {!Blockly.Workspace} workspace The flyout's workspace.
 */
Blockly.Procedures.flyoutCategory = function(blocks, gaps, margin, workspace) {
  if (Blockly.Blocks['thymio_subroutine_define']) {
    var block = Blockly.Block.obtain(workspace, 'thymio_subroutine_define');
    block.initSvg();
    blocks.push(block);
    gaps.push(margin * 2);
  }
  if (gaps.length) {
    // Add slightly larger gap between system blocks and user calls.
    gaps[gaps.length - 1] = margin * 3;
  }

  function populateProcedures(procedureList, templateName) {
    for (var x = 0; x < procedureList.length; x++) {
      var block = Blockly.Block.obtain(workspace, templateName);
      block.setFieldValue(procedureList[x][0], 'NAME');
      var tempIds = [];
      for (var t = 0; t < procedureList[x][1].length; t++) {
        tempIds[t] = 'ARG' + t;
      }
      block.setProcedureParameters(procedureList[x][1], tempIds);
      block.initSvg();
      blocks.push(block);
      gaps.push(margin * 2);
    }
  }

  var tuple = Blockly.Procedures.allProcedures(workspace.targetWorkspace);
  populateProcedures(tuple[0], 'procedures_callnoreturn');
};
