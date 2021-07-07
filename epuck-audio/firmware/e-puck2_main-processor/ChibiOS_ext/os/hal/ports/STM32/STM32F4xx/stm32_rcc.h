/**
 * @name    DCMI peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DCMI peripheral clock.
 *
 * @param[in] lp     low power enable flag
 *
 * @api
 */
#define rccEnableDCMI(lp) rccEnableAHB2(RCC_AHB2ENR_DCMIEN, lp)

/**
 * @brief   Disables the DCMI peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableDCMI(lp) rccDisableAHB2(RCC_AHB2ENR_DCMIEN, lp)

/**
 * @brief    Resets the DCMI periphera.
 *
 * @api
 */
#define rccResetDCMI() rccResetAHB2(RCC_AHB2RSTR_DCMIRST)
