#include "pdm2pcm.h"
#include <string.h>

// Static memory used by the filter (required by ST)
static uint8_t pdm2pcm_memory[PDM2PCM_MEMORY_SIZE];

uint32_t PDM_Filter_Init(PDM2PCM_Handler_t *handler)
{
    if (!handler) return PDM2PCM_INIT_ERROR;

    memset(handler, 0, sizeof(PDM2PCM_Handler_t));

  
  handler->bit_order     = PDM2PCM_BIT_ORDER_LSB;
    handler->endianness    = PDM2PCM_ENDIANNESS_LE;
    handler->high_pass_tap = 2104533974; // Recommended default
    handler->internal_memory = pdm2pcm_memory;

    return PDM2PCM_OK;
}



uint32_t PDM_Filter_setConfig(PDM2PCM_Handler_t *handler, PDM2PCM_Config_t *config)
{
    if (!handler || !config)
        return PDM2PCM_CONFIG_ERROR;

    // Validate configuration
    if ((config->decimation_factor != PDM2PCM_DEC_FACTOR_64) &&
        (config->decimation_factor != PDM2PCM_DEC_FACTOR_128))
        return PDM2PCM_DECIMATION_ERROR;

    return PDM2PCM_OK;
}

uint32_t PDM_Filter(void *pdm_data, void *pcm_data, PDM2PCM_Handler_t *handler)
{
    if (!pdm_data || !pcm_data || !handler)
        return PDM2PCM_INIT_ERROR;

    // Cast to correct types
    uint8_t *pdm = (uint8_t *)pdm_data;
    int16_t *pcm = (int16_t *)pcm_data;

    // Create local config
    PDM2PCM_Config_t config;
    config.decimation_factor     = PDM2PCM_DEC_FACTOR_64;
    config.output_samples_number = 128;  // Adjust to match your application
    config.mic_gain              = 6;   // Recommended
    config.bit_depth             = PDM2PCM_BITDEPTH_16;

    // Apply config and process
    PDM_Filter_setConfig(handler, &config);

    return PDM_Filter_64(pdm, pcm, handler, &config);  // ST's internal function
}
