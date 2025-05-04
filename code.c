//
// kernel.c
//
// Copyright (C) 2024 Macoy Madson <macoy@macoy.me>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "kernel.h"

#include "circle/memory.h"
#include "circle/machineinfo.h"

#include "circle/util.h"

#define V3D_IMPLEMENTATION
#include "v3d/v3d.h"
#undef V3D_IMPLEMENTATION

#include <circle/2dgraphics.h>
#include <circle/actled.h>
#include <circle/exceptionhandler.h>
#include <circle/koptions.h>
#include <circle/logger.h>
#include <circle/screen.h>
#include <circle/serial.h>
#include <circle/types.h>

#define v3d_memcmp memcmp
#define v3d_vsnprintf SString_FormatV
#define v3d_assert assert

#define V3D_ASSEMBLER_IMPLEMENTATION
#include "/media/macoy/Preserve/dev/rpi-bare-metal/tools/v3dAssembler.h"
#undef V3D_ASSEMBLER_IMPLEMENTATION

C2DGraphics		m_2DGraphics;
CLogger		m_Logger;

// Working: 59000, 59250, 59375, 59437, 59468, 59483, 59495, 59500, 59960, 59965
// Broken: 60000, 59500, 59975, 59970, 59968, 59967, 59966
// 59965 working
// 59966 broken... WHY?
#define NUM_TRIANGLES 59965

static float g_instance_offsets[NUM_TRIANGLES * 2];

boolean Kernel_Initialize (void)
{
	CLogger_Create(&m_Logger, KernelOptions_GetLogLevel(), /*bOverwriteOldest=*/TRUE);
	Timer_Create();
	Timer_Initialize();
	// No target
	CLogger_Initialize (&m_Logger, 0);
	return TRUE;
}

static u32 divideRoundUp(u32 a, u32 b)
{
    return (a + b - 1) / b;
}

static void getSuperTiles(u32* supertile_w, u32* supertile_h,
                          u32* frame_w_supertile, u32* frame_h_supertile,
                          u32 tiles_x, u32 tiles_y)
{
    const u32 max_supertiles = 256;

    (*supertile_w) = 1;
	(*supertile_h) = 1;

    while (TRUE)
    {
        (*frame_w_supertile) = divideRoundUp(tiles_x, (*supertile_w));
		(*frame_h_supertile) = divideRoundUp(tiles_y, (*supertile_h));

        if ((*frame_w_supertile) * (*frame_h_supertile) < max_supertiles)
            break;

        if ((*supertile_w) < (*supertile_h))
            (*supertile_w)++;
        else
            (*supertile_h)++;
    }
}

/*
#version 300 es
precision highp float;

in vec4 v_color;

out vec4 outColor;

void main()
{
    outColor = v_color;
}
*/

static const char* g_fragment_shader_assembly[] = {
    // payload_w : rf0
    // payload_w_centroid : rf1
    // payload_z : rf2

    "nop                           ; nop ; ldvary.r0", // 0x3D103186BB800000
    "nop                           ; fmul r1, r0, rf0", // 0x54003046BBC00000
    "fadd rf6, r1, r5              ; nop ; ldvary.r2", // 0x3D10A18605829000
    "nop                           ; fmul r3, r2, rf0", // 0x540030C6BBC80000
    "fadd rf5, r3, r5              ; nop ; ldvary.r4", // 0x3D1121850582B000
    "nop                           ; fmul r0, r4, rf0", // 0x54003006BBD00000
    "fadd rf4, r0, r5              ; nop ; thrsw; ldvary.r1", // 0x3D30618405828000
    "nop                           ; fmul r2, r1, rf0 ; thrsw", // 0x54203086BBC40000
    "fadd rf3, r2, r5              ; nop", // 0x3C0021830582A000
    "vfpack tlb, rf6, rf5          ; nop ; thrsw", // 0x3C2031873583E185
    "vfpack tlb, rf4, rf3          ; nop", // 0x3C0031873583E103
    "nop                           ; nop", // 0x3C003186BB800000

    // out[0] = vary[0] * payload_w + r5
    // out[1] = vary[1] * payload_w + r5
    // out[2] = vary[2] * payload_w + r5
    // out[3] = vary[3] * payload_w + r5
};

static const char* g_vertex_shader_assembly[] = {
	// xyz
    "ldvpmv_in rf3, 0              ; nop", // 0x3DE02187BC807000
    "ldvpmv_in rf4, 1              ; nop", // 0x3DE02189BC807001
    "ldvpmv_in rf5, 2             ; nop", // 0x3DE0218ABC807002
	// color rgba
    "ldvpmv_in rf9, 3              ; nop", // 0x3DE02183BC807003
	"ldvpmv_in rf10, 4              ; nop", // 0x3DE02184BC807004
	"ldvpmv_in rf11, 5              ; nop", // 0x3DE02184BC807004
	"ldvpmv_in rf12, 6              ; nop", // 0x3DE02184BC807004
	// offset XY
	"ldvpmv_in rf7, 7              ; nop", // 0x3DE02186BC807006
	"ldvpmv_in rf8, 8              ; nop", // 0x3DE02186BC807006
	// Load w
	"nop                           ; nop ; ldunifrf.rf13", // 0x3D823186BB800000
	// X + offset X ; Load viewport X scale
	"fadd r0, rf3, rf7             ; nop ; ldunif", // 0x3D823186BB800000
	/* "fadd r0, rf3, rf8             ; nop ; ldunif", // Plus Y instead */
	// X * viewport X, load viewport Y
	"nop ; fmul r1, r0, r5 ; ldunif",
	// X to int
	"ftoiz r0, r1 ; nop",
	// Store screen position X
	"stvpmv 0, r0 ; nop",

	// Y + offset Y ;
	"fadd r0, rf4, rf8 ; nop",
	// Y * viewport Y, load viewport Z
	"nop ; fmul r1, r0, r5 ; ldunif",
	/* "nop ; fmul r1, rf4, r5 ; ldunif", */
	// Y to int ;
	"ftoiz r0, r1 ; nop",
	// Store screen position Y
	"stvpmv 1, r0 ; nop",

	// Z * viewport Z, load viewport Z offset
	"nop ; fmul r0, rf5, r5 ; ldunif",
	// Z + viewport Z offset
	"fadd r1, r0, r5 ; nop",
	// Store Z (Zs)
	"stvpmv 2, r1 ; nop",
	// Store 1/Wc
    "stvpmv 3, rf13                 ; nop",

	// Store RGBA
	"stvpmv 4, rf9                 ; nop",
	"stvpmv 5, rf10                 ; nop",
	"stvpmv 6, rf11                 ; nop",
	"stvpmv 7, rf12                 ; nop",

	// Finished
    "vpmwt -                       ; nop",
    "nop                           ; nop ; thrsw",
    "nop                           ; nop",
    "nop                           ; nop",
};

static const char* g_coordinate_shader_assembly[] = {
	// Load XYZ
    "ldvpmv_in rf4, 0              ; nop",
    "ldvpmv_in rf5, 1              ; nop",
    "ldvpmv_in rf3, 2              ; nop",
	// Load offset XY
	"ldvpmv_in rf6, 3              ; nop",
	"ldvpmv_in rf7, 4              ; nop",

	// X + X offset
	"fadd r3, rf4, rf6             ; nop",
	/* "fadd r3, rf4, rf7             ; nop", // Plus y offset instead */
    "stvpmv 0, r3                  ; nop",
	// Y + Y offset
    "fadd r2, rf5, rf7             ; nop",
	"stvpmv 1, r2                  ; nop",
	/* "stvpmv 1, rf5                  ; nop", */
	// Load W
    "nop                           ; nop ; ldunif",
    "stvpmv 2, rf3                 ; nop",
    "stvpmv 3, r5                  ; nop",
    "nop                           ; nop ; ldunif",
	// X * viewport X scale ; load viewport Y scale
    "nop                           ; fmul r0, r3, r5 ; ldunif",
	// X to int ; Y * viewport Y scale
    "ftoiz r1, r0                  ; fmul r3, r2, r5",
    "stvpmv 4, r1                  ; nop",
    "ftoiz r2, r3                  ; nop",
    "stvpmv 5, r2                  ; nop",

    // Finished
    "vpmwt -                       ; nop",
    "nop                           ; nop ; thrsw",
    "nop                           ; nop",
    "nop                           ; nop",

    // X, Y, Z, W, Xs, Ys
    // pos
    // out[0] = in[0]
    // out[1] = in[1]
    // out[2] = in[2]
    // out[3] = unif[0]
    // vp
    // out[4] = (int)(in[0] * unif[1])
    // out[5] = (int)(in[1] * unif[2])
};

static v3d_qpu_instruction g_fragment_shader_buffer[64] = {0};
static struct v3d_qpu_instr g_fragment_shader_unpacked_instructions[64] = {0};
static int g_fragment_shader_buffer_num_instructions = 0;
static v3d_qpu_instruction g_vertex_shader_buffer[64] = {0};
static struct v3d_qpu_instr g_vertex_shader_unpacked_instructions[64] = {0};
static int g_vertex_shader_buffer_num_instructions = 0;
static v3d_qpu_instruction g_coordinate_shader_buffer[64] = {0};
static struct v3d_qpu_instr g_coordinate_shader_unpacked_instructions[64] = {0};
static int g_coordinate_shader_buffer_num_instructions = 0;

static float __attribute__((aligned(64))) g_vertex_positions[] = {
    -1.f, -1.f, 0.f,
	1.f, -1.f, 0.f,
	0.f, 1.f, 0.f,

	0.f, 0.f, 0.f,
	0.25f, 0.f, 0.f,
	0.25f, 0.25f, 0.f,
};

static u8 __attribute__((aligned(64))) g_vertex_colors_rgba[] = {
    0xFF, 0x00, 0x00, 0xFF,
	0x00, 0xFF, 0x00, 0xFF,
	0x00, 0x00, 0xFF, 0xFF,

	0xFF, 0xFF, 0x00, 0xFF,
	0x00, 0xFF, 0x00, 0xFF,
	0x00, 0xFF, 0xFF, 0xFF,
};

// C3D_PCS V3D pipeline control and status
#define V3D_CLE_PCS (V3D_CORE0 + 0x130)
// Binning mode out of memory
#define V3D_CLE_PCS_BMOOM (BIT(8))
// Rendering mode busy
#define V3D_CLE_PCS_RMBUSY (BIT(3))
// Rendering mode active
#define V3D_CLE_PCS_RMACTIVE (BIT(2))
// Binning mode busy
#define V3D_CLE_PCS_BMBUSY (BIT(1))
// Binning mode active
#define V3D_CLE_PCS_BMACTIVE (BIT(0))

#define V3D_ERR_STAT (V3D_CORE0 + 0xf20)
#define V3D_ERR_L2CARE    (BIT(15))
#define V3D_ERR_VCMBE     (BIT(14))
#define V3D_ERR_VCMRE     (BIT(13))
#define V3D_ERR_VCDI      (BIT(12))
#define V3D_ERR_VCDE      (BIT(11))
#define V3D_ERR_VDWE      (BIT(10))
#define V3D_ERR_VPMEAS    (BIT(9))
#define V3D_ERR_VPMEFNA   (BIT(8))
#define V3D_ERR_VPMEWNA   (BIT(7))
#define V3D_ERR_VPMERNA   (BIT(6))
#define V3D_ERR_VPMERR    (BIT(5))
#define V3D_ERR_VPMEWR    (BIT(4))
#define V3D_ERR_VPAERRGL  (BIT(3))
#define V3D_ERR_VPAEBRGL  (BIT(2))
#define V3D_ERR_VPAERGS   (BIT(1))
#define V3D_ERR_VPAEABB   (BIT(0))

u32 unusedThing = 0;

#include "circle/memio.h"
void debugRegisters(void)
{
	u32 commandThread0Status = read32(V3D_CLE_CT0CS);
	u32 errorStatus = read32(V3D_ERR_STAT);
	u32 pipelineStatus = read32(V3D_CLE_PCS);
	// Just to make sure this code ends up generated.
	unusedThing = commandThread0Status + errorStatus + pipelineStatus;
}

#define HEAP_LOW 0

static char static_heap[1024 * 1024 * 100];
v3d_static_buffer static_heap_buffer = {};

void* Static_HeapAllocate(size_t size, int ignored)
{
	if (!static_heap_buffer.start)
	{
		static_heap_buffer.start = (u8*)static_heap;
		static_heap_buffer.capacity = sizeof(static_heap);
	}
	const int defaultAlign = 4096; // I believe this is unnecessarily large (todo shrink)
	v3d_buffer_align(&static_heap_buffer, defaultAlign);
	void* data = v3d_buffer_claim_memory(&static_heap_buffer, size);
	assert(data);
	return data;
}

void Static_HeapFree(void* mem)
{
	// Do nothing for now
}

typedef struct PrepareShaderStateRecordArguments
{
	int renderWidth;
	int renderHeight;
	u8* fragment_shader_buffer;
	int fragment_shader_buffer_size;

	u8* vertex_shader_buffer;
	int vertex_shader_buffer_size;

	u8* coordinate_shader_buffer;
	int coordinate_shader_buffer_size;

	u8* vertex_positions;
	u8* vertex_colors_rgba;
} PrepareShaderStateRecordArguments;

v3d_gl_shader_state_record* prepareShaderStateRecord(PrepareShaderStateRecordArguments* args, int* numAttributesOut, u8** invalidateCacheStart, u64* invalidateCacheSize)
{
	if (!args->fragment_shader_buffer)
	{
		args->fragment_shader_buffer = (u8*)g_fragment_shader_buffer;
	}
	if (!args->fragment_shader_buffer_size)
	{
		args->fragment_shader_buffer_size = sizeof(g_fragment_shader_buffer);
	}
	if (!args->vertex_shader_buffer)
	{
		args->vertex_shader_buffer = (u8*)g_vertex_shader_buffer;
	}
	if (!args->vertex_shader_buffer_size)
	{
		args->vertex_shader_buffer_size = sizeof(g_vertex_shader_buffer);
	}
	if (!args->coordinate_shader_buffer)
	{
		args->coordinate_shader_buffer = (u8*)g_coordinate_shader_buffer;
	}
	if (!args->coordinate_shader_buffer_size)
	{
		args->coordinate_shader_buffer_size = sizeof(g_coordinate_shader_buffer);
	}

	if (!args->vertex_positions)
	{
		args->vertex_positions = (u8*)g_vertex_positions;
	}
	if (!args->vertex_colors_rgba)
	{
		args->vertex_colors_rgba = (u8*)g_vertex_colors_rgba;
	}

	static u8 __attribute__((aligned(64))) defaultAttributesStateBuffer[4096] = {0};
	v3d_static_buffer defaultAttributesState = {defaultAttributesStateBuffer, 0,
	                                            sizeof(defaultAttributesStateBuffer)};

	// Uniforms
	u8* fragmentUniformsAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	u8* vertexUniformsAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	// Screen position is 24.8 fixed point
	float toFixedPoint24dot8 = 256.f;
	struct
	{
		float w;
		float viewportXScale;
		float viewportYScale;
		float viewportZScale;
		float viewportZOffset;
	} vertexUniform = {1.f, (args->renderWidth / 2) * toFixedPoint24dot8,
	                   (args->renderHeight / 2) * -toFixedPoint24dot8, 0.5f, 0.5f};
	v3d_buffer_write(&defaultAttributesState, &vertexUniform, sizeof(vertexUniform));
	u8* coordinateUniformsAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	struct
	{
		float w;
		float viewportXScale;
		float viewportYScale;
	} coordinateUniform = {1.f, (args->renderWidth / 2) * toFixedPoint24dot8,
	                       (args->renderHeight / 2) * -toFixedPoint24dot8};
	v3d_buffer_write(&defaultAttributesState, &coordinateUniform, sizeof(coordinateUniform));

	// Default attributes
	u8* defaultAttributesAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	float defaultInputFloats[4] = {0.f, 0.f, 0.f, 1.f};
	// (todo understanding) I'm not sure where this number comes from
	int V3D_MAX_VS_INPUTS = 64;
	for (int i = 0; i < V3D_MAX_VS_INPUTS / (int)ArraySize(defaultInputFloats); ++i)
	{
		v3d_buffer_write(&defaultAttributesState, defaultInputFloats, sizeof(defaultInputFloats));
	}
	v3d_buffer_align(&defaultAttributesState, 8);
	u8* fragmentShaderAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	v3d_buffer_write(&defaultAttributesState, args->fragment_shader_buffer,
	                 args->fragment_shader_buffer_size);
	v3d_buffer_align(&defaultAttributesState, 8);
	u8* vertexShaderAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	v3d_buffer_write(&defaultAttributesState, args->vertex_shader_buffer,
	                 args->vertex_shader_buffer_size);
	v3d_buffer_align(&defaultAttributesState, 8);
	u8* coordinateShaderAddress = V3D_BUFFER_WRITE_HEAD(defaultAttributesState);
	v3d_buffer_write(&defaultAttributesState, args->coordinate_shader_buffer,
	                 args->coordinate_shader_buffer_size);

	// GL_SHADER_STATE_RECORD requires 32 byte alignment
	v3d_buffer_align(&defaultAttributesState, 32);
	v3d_gl_shader_state_record* shaderStateRecord =
	    V3D_BUFFER_ALLOC_STRUCT(&defaultAttributesState, v3d_gl_shader_state_record);
	assert(shaderStateRecord);
	shaderStateRecord->enable_clipping = TRUE;
	shaderStateRecord->fragment_shader_uses_real_pixel_centre_w_in_addition_to_centroid_w2 = TRUE;
	shaderStateRecord->disable_implicit_point_line_varyings = TRUE;
	shaderStateRecord->number_of_varyings_in_fragment_shader = 4;  // 4 component color
	// NOTE: Setting the segment sizes below the attributes!
	shaderStateRecord->address_of_default_attribute_values =
	    V3D_ARM_TO_BUS_ADDR(defaultAttributesAddress);

	shaderStateRecord->fragment_shader_code_address_rshift_3 =
	    V3D_ARM_TO_BUS_ADDR(fragmentShaderAddress) >> 3;
	shaderStateRecord->fragment_shader_uniforms_address =
	    V3D_ARM_TO_BUS_ADDR(fragmentUniformsAddress);
	shaderStateRecord->fragment_shader_4_way_threadable = TRUE;
	shaderStateRecord->fragment_shader_start_in_final_thread_section = FALSE;
	shaderStateRecord->fragment_shader_propagate_nans = TRUE;

	shaderStateRecord->vertex_shader_code_address_rshift_3 =
	    V3D_ARM_TO_BUS_ADDR(vertexShaderAddress) >> 3;
	shaderStateRecord->vertex_shader_uniforms_address = V3D_ARM_TO_BUS_ADDR(vertexUniformsAddress);
	shaderStateRecord->vertex_shader_4_way_threadable = TRUE;
	shaderStateRecord->vertex_shader_start_in_final_thread_section = TRUE;
	shaderStateRecord->vertex_shader_propagate_nans = TRUE;

	shaderStateRecord->coordinate_shader_code_address_rshift_3 =
	    V3D_ARM_TO_BUS_ADDR(coordinateShaderAddress) >> 3;
	shaderStateRecord->coordinate_shader_uniforms_address =
	    V3D_ARM_TO_BUS_ADDR(coordinateUniformsAddress);
	shaderStateRecord->coordinate_shader_4_way_threadable = TRUE;
	shaderStateRecord->coordinate_shader_start_in_final_thread_section = TRUE;
	shaderStateRecord->coordinate_shader_propagate_nans = TRUE;

	// Gl shader attribute records immediately follow the v3d_gl_shader_state_record, which is
	// technically variable-length according to how many attributes there are. In the "Shader State
	// Record Formats" of VideoCoreIV-AG100-R VideoCore IV 3D Architecture Reference Guide they are
	// actually written as if they are inside the record.
	int numAttributes = 0;
	v3d_gl_shader_state_attribute_record* positionAttribute = V3D_BUFFER_ALLOC_STRUCT(
	    &defaultAttributesState, v3d_gl_shader_state_attribute_record);
	assert(positionAttribute);
	positionAttribute->address = V3D_ARM_TO_BUS_ADDR(args->vertex_positions);
	positionAttribute->number_of_values_read_by_vertex_shader = 3;
	positionAttribute->number_of_values_read_by_coordinate_shader = 3;
	positionAttribute->instance_divisor = 0;
	positionAttribute->stride = 3 * sizeof(float);
	positionAttribute->maximum_index = 0xFFFFFF;
	positionAttribute->vec_size = v3d_VEC_3;
	positionAttribute->type = v3d_ATTRIBUTE_FLOAT;
	++numAttributes;

	v3d_gl_shader_state_attribute_record* colorAttribute = V3D_BUFFER_ALLOC_STRUCT(
	    &defaultAttributesState, v3d_gl_shader_state_attribute_record);
	assert(colorAttribute);
	colorAttribute->address = V3D_ARM_TO_BUS_ADDR(args->vertex_colors_rgba);
	colorAttribute->number_of_values_read_by_vertex_shader = 4;
	colorAttribute->number_of_values_read_by_coordinate_shader = 0;
	colorAttribute->instance_divisor = 1;
	colorAttribute->stride = 4 * sizeof(u8);
	colorAttribute->maximum_index = 0xFFFFFF;
	colorAttribute->type = v3d_ATTRIBUTE_BYTE;
	colorAttribute->normalized_int_type = TRUE;
	++numAttributes;

	v3d_gl_shader_state_attribute_record* offsetAttribute = V3D_BUFFER_ALLOC_STRUCT(
	    &defaultAttributesState, v3d_gl_shader_state_attribute_record);
	assert(offsetAttribute);
	offsetAttribute->address = V3D_ARM_TO_BUS_ADDR(g_instance_offsets);
	offsetAttribute->number_of_values_read_by_vertex_shader = 2;
	offsetAttribute->number_of_values_read_by_coordinate_shader = 2;
	offsetAttribute->instance_divisor = 1;
	offsetAttribute->stride = 2 * sizeof(float);
	offsetAttribute->maximum_index = 0xFFFFFF;
	offsetAttribute->type = v3d_ATTRIBUTE_FLOAT;
	offsetAttribute->vec_size = v3d_VEC_2;
	++numAttributes;

	// Now let's tell the VPM how much memory we need to allocate
	int coordinateNumValues = 0;
	int vertexNumValues = 0;
	for (int attributeIndex = 0; attributeIndex < numAttributes; ++attributeIndex)
	{
		v3d_gl_shader_state_attribute_record* attribute = &positionAttribute[attributeIndex];
		// Assume 4 bytes per value
		coordinateNumValues += attribute->number_of_values_read_by_coordinate_shader;
		vertexNumValues += attribute->number_of_values_read_by_vertex_shader;
	}
	shaderStateRecord->coordinate_shader_output_vpm_segment_size = V3D_ALIGN(coordinateNumValues, 8) / 8;
	shaderStateRecord->coordinate_shader_input_vpm_segment_size = 1;
	// See v3d_vs_set_prog_data()
	shaderStateRecord->vertex_shader_output_vpm_segment_size = V3D_ALIGN(vertexNumValues, 8) / 8;
	// Even though the shader may have more than this in input, I think with non-separate sections
	// the output segment effectively determines the input segment size. I'm not sure this 1 here
	// matters; it could likely be 0 and work fine.
	shaderStateRecord->vertex_shader_input_vpm_segment_size = 1;

	assert(!v3d_buffer_out_of_memory(&defaultAttributesState));

	if (invalidateCacheStart)
	{
		*invalidateCacheStart = defaultAttributesState.start;
		*invalidateCacheSize = defaultAttributesState.used;
	}
	*numAttributesOut = numAttributes;
	return shaderStateRecord;
}

void prepareBinningCommandList(v3d_static_buffer* binningCommandList, int renderWidth,
                               int renderHeight, int numAttributes,
                               v3d_gl_shader_state_record* shaderStateRecord, int vertexCount,
                               u8* compareBuffer)
{
	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_TILE_BINNING_MODE_CFG,
	                           v3d_tile_binning_mode_cfg, binningModeConfig);
	assert(binningModeConfig);
	binningModeConfig->tile_allocation_initial_block_size = 0;
	binningModeConfig->tile_allocation_block_size = 0;
	binningModeConfig->number_of_render_targets_minus_one = 1 - 1;
	binningModeConfig->maximum_bpp_of_all_render_targets = V3D_INTERNAL_BPP_32;
	binningModeConfig->multisample_mode_4x = FALSE;
	binningModeConfig->double_buffer_in_non_ms_mode = FALSE;
	binningModeConfig->width_in_pixels_minus_one = renderWidth - 1;
	binningModeConfig->height_in_pixels_minus_one = renderHeight - 1;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList, v3d_OP_FLUSH_VCD_CACHE,
	                                        v3d_flush_vcd_cache);

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	{
		V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_OCCLUSION_QUERY_COUNTER,
		                           v3d_occlusion_query_counter, occlusionQueryCounter);
		assert(occlusionQueryCounter);
		occlusionQueryCounter->address = 0;
	}

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList, v3d_OP_START_TILE_BINNING,
	                                        v3d_start_tile_binning);

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_CLIPWINDOW, v3d_clipwindow, clipWindow);
	assert(clipWindow);
	clipWindow->clip_window_left_pixel_coordinate = 0;
	clipWindow->clip_window_bottom_pixel_coordinate = 0;
	clipWindow->clip_window_width_in_pixels = renderWidth;
	clipWindow->clip_window_height_in_pixels = renderHeight;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_CFG_BITS, v3d_cfg_bits, cfgBits);
	assert(cfgBits);
	cfgBits->enable_forward_facing_primitive = TRUE;
	cfgBits->enable_reverse_facing_primitive = TRUE;
	cfgBits->clockwise_primitives = TRUE;
	cfgBits->enable_depth_offset = FALSE;
	cfgBits->line_rasterization = V3D_LINE_RASTERIZATION_DIAMOND_EXIT;
	cfgBits->rasterizer_oversample_mode = 0;
	cfgBits->direct3d_wireframe_triangles_mode = FALSE;
	cfgBits->depth_test_function = V3D_COMPARE_FUNC_LESS;
	cfgBits->z_updates_enable = FALSE;
	cfgBits->early_z_enable = FALSE;
	cfgBits->early_z_updates_enable = TRUE;
	cfgBits->stencil_enable = FALSE;
	cfgBits->blend_enable = FALSE;
	cfgBits->direct3d_point_fill_mode = FALSE;
	cfgBits->direct3d_provoking_vertex = FALSE;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_POINT_SIZE, v3d_point_size, pointSize);
	assert(pointSize);
	pointSize->point_size = 1.f;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_LINE_WIDTH, v3d_line_width, lineWidth);
	assert(lineWidth);
	lineWidth->line_width = 1.f;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_CLIPPER_XY_SCALING,
	                           v3d_clipper_xy_scaling, ClipperXYScaling);
	assert(ClipperXYScaling);
	ClipperXYScaling->viewport_half_width_in_1_256th_of_pixel = (renderWidth / 2) * 256.f;
	ClipperXYScaling->viewport_half_height_in_1_256th_of_pixel = (renderHeight / 2) * -256.f;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_CLIPPER_Z_SCALE_AND_OFFSET,
	                           v3d_clipper_z_scale_and_offset, ClipperZScaleAndOffset);
	assert(ClipperZScaleAndOffset);
	ClipperZScaleAndOffset->viewport_z_scale_zc_to_zs = 0.5f;
	ClipperZScaleAndOffset->viewport_z_offset_zc_to_zs = 0.5f;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_CLIPPER_Z_MIN_MAX_CLIPPING_PLANES,
	                           v3d_clipper_z_min_max_clipping_planes, ClipperZMinMaxClippingPlanes);
	assert(ClipperZMinMaxClippingPlanes);
	ClipperZMinMaxClippingPlanes->minimum_zw = 0.f;
	ClipperZMinMaxClippingPlanes->maximum_zw = 1.f;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_VIEWPORT_OFFSET, v3d_viewport_offset,
	                           ViewportOffset);
	assert(ViewportOffset);
	ViewportOffset->fine_x = V3D_FLOAT_TO_U14_8((float)renderWidth / 2.f);
	ViewportOffset->coarse_x = 0;
	ViewportOffset->fine_y = V3D_FLOAT_TO_U14_8((float)renderHeight / 2.f);
	ViewportOffset->coarse_y = 0;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_COLOR_WRITE_MASKS, v3d_color_write_masks,
	                           ColorWriteMasks);
	assert(ColorWriteMasks);
	ColorWriteMasks->mask = 0;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_BLEND_CONSTANT_COLOR,
	                           v3d_blend_constant_color, BlendConstantColor);
	assert(BlendConstantColor);
	BlendConstantColor->red_f16 = 0;
	BlendConstantColor->green_f16 = 0;
	BlendConstantColor->blue_f16 = 0;
	BlendConstantColor->alpha_f16 = 0;

	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList, v3d_OP_ZERO_ALL_FLAT_SHADE_FLAGS,
	                                        v3d_zero_all_flat_shade_flags);
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}
	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList,
	                                        v3d_OP_ZERO_ALL_NON_PERSPECTIVE_FLAGS,
	                                        v3d_zero_all_non_perspective_flags);
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}
	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList, v3d_OP_ZERO_ALL_CENTROID_FLAGS,
	                                        v3d_zero_all_centroid_flags);
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_TRANSFORM_FEEDBACK_SPECS,
	                           v3d_transform_feedback_specs, TransformFeedbackSpecs);
	assert(TransformFeedbackSpecs);
	TransformFeedbackSpecs->number_of_16_bit_output_data_specs_following = 0;
	TransformFeedbackSpecs->enable = FALSE;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	{
		V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_OCCLUSION_QUERY_COUNTER,
		                           v3d_occlusion_query_counter, OcclusionQueryCounter);
		assert(OcclusionQueryCounter);
		OcclusionQueryCounter->address = 0;
	}
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_SAMPLE_STATE, v3d_sample_state,
	                           SampleState);
	assert(SampleState);
	SampleState->mask = 0xF;
	SampleState->coverage = v3d_float_to_f187(1.0f);
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_VCM_CACHE_SIZE, v3d_vcm_cache_size,
	                           VcmCacheSize);
	assert(VcmCacheSize);
	VcmCacheSize->number_of_16_vertex_batches_for_binning = 4;
	VcmCacheSize->number_of_16_vertex_batches_for_rendering = 4;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_GL_SHADER_STATE, v3d_gl_shader_state,
	                           GlShaderState);
	assert(GlShaderState);
	GlShaderState->number_of_attribute_arrays = numAttributes;
	GlShaderState->address_rshift_5 = V3D_ARM_TO_BUS_ADDR(shaderStateRecord) >> 5;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(binningCommandList, v3d_OP_VERTEX_ARRAY_INSTANCED_PRIMS, v3d_vertex_array_instanced_prims,
	                           VertexArrayPrims);
	assert(VertexArrayPrims);
	VertexArrayPrims->mode = V3D_PRIM_TRIANGLES;
	VertexArrayPrims->instance_length = 3;
	VertexArrayPrims->index_of_first_vertex = 0;
	VertexArrayPrims->number_of_instances = NUM_TRIANGLES;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}

	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(binningCommandList, v3d_OP_FLUSH,
	                                        v3d_flush);
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, binningCommandList->start, binningCommandList->used) == 0);
	}
}

void* compareWithBuffer(int renderWidth, int renderHeight,
					   int numAttributes, void* shaderStateRecord, int vertexCount,
						void* bufferToCompare, int* overrideBCLLength)
{
	static u8 __attribute__((aligned(64))) binningCommandListBuffer[4096] = {0};
	v3d_static_buffer binningCommandList = {binningCommandListBuffer, 0,
	                                        sizeof(binningCommandListBuffer)};
	prepareBinningCommandList(&binningCommandList, renderWidth, renderHeight, numAttributes,
							  (v3d_gl_shader_state_record*)shaderStateRecord, vertexCount, (u8*)bufferToCompare);
	*overrideBCLLength = binningCommandList.used;
	return binningCommandList.start;
}

typedef struct GetTileStatesArguments
{
	u8* tileAllocation;
	size_t tileAllocationSize;
	u8* tileState;
	size_t tileStateSize;
} GetTileStatesArguments;
void getTileStates(GetTileStatesArguments* args, int renderWidth, int renderHeight,
				   int sizeofVertexPositions, int sizeofVertexColors)
{
	if (!sizeofVertexPositions)
	{
		sizeofVertexPositions = sizeof(g_vertex_positions);
	}
	if (!sizeofVertexColors)
	{
		sizeofVertexColors = sizeof(g_vertex_colors_rgba);
	}
// decreases if msaa/double buffering/multiple color attachement/more bpp
	int tileWidth = 64;
	int tileHeight = 64;
	int tilesX = divideRoundUp(renderWidth, tileWidth);
    int tilesY = divideRoundUp(renderHeight, tileHeight);

	// Comments from mesa/src/broadcom/vulkan/v3dv_cmd_buffer.c (MIT) v3dv_job_allocate_tile_state()
	// The PTB will request the tile alloc initial size per tile at start of tile binning.
	int numLayers = 1;
	size_t tileAllocationSize = numLayers * tilesX * tilesY * 64;
	// The PTB allocates in aligned 4k chunks after the initial setup.
	tileAllocationSize = (size_t)v3d_get_aligned_address((void*)tileAllocationSize, 4096);
	// Include the first two chunk allocations that the PTB does so that
    // we definitely clear the OOM condition before triggering one (the HW
    // won't trigger OOM during the first allocations).
	tileAllocationSize += 8192;
	// For performance, allocate some extra initial memory after the PTB's
    // minimal allocations, so that we hopefully don't have to block the
    // GPU on the kernel handling an OOM signal.
	tileAllocationSize += 512 * 1024;
	// // (todo) Determine this relationship
	// size_t macoy_fudge_factor = 2; // 2 is enough for 100k triangles
	// tileAllocationSize += (sizeofVertexPositions + sizeofVertexColors) * macoy_fudge_factor;

	// Macoy: The above is enough to get through binning, but rendering needs more. Let's add another 512 kiB
	tileAllocationSize += 512 * 1024;

	// (todo check alignment) -- this.
	// Needs to be in range of the GPU, but I think that might mean anything <3gib, so HIGH might
	// work too
	void* tileAllocation = Static_HeapAllocate(tileAllocationSize, HEAP_LOW);
	const int tileStateDataArrayPerTileSize = 256;
	args->tileStateSize = numLayers * tilesX * tilesY * tileStateDataArrayPerTileSize;
	void* tileState = Static_HeapAllocate(args->tileStateSize, HEAP_LOW);

	args->tileAllocation = (u8*)tileAllocation;
	args->tileAllocationSize = tileAllocationSize;
	args->tileState = (u8*)tileState;
}

typedef struct PrepareRenderCommandListArguments
{
	int renderWidth;
	int renderHeight;
	int tilesX;
	int tilesY;
	int tileWidth;
	int tileHeight;
	int renderTargetStride;
	u8* startRenderTarget;
	int zBufferStride;
	u8* zBuffer;
	u8* tileAllocation;
	// Only used with compareBuffer, does mean indirect built here isn't used in compare
	u32 indirectStart;
	u32 indirectEnd;
} PrepareRenderCommandListArguments;
void prepareRenderCommandList(v3d_static_buffer* renderCommandList,
                              v3d_static_buffer* indirectTileRenderCommandList,
                              PrepareRenderCommandListArguments* args, u8* compareBuffer,
                              u8* indirectCompareBuffer, u8** invalidateCacheStart,
                              u64* invalidateCacheSize)
{
	u8* indirectTileRenderCommandListStartCommands = 0;

	// Must be first
	V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_RENDERING_MODE_CFG_COMMON,
							   v3d_tile_rendering_mode_cfg_common, TileRenderingModeCfgCommon);
	assert(TileRenderingModeCfgCommon);
	TileRenderingModeCfgCommon->sub_id = 0;
	TileRenderingModeCfgCommon->number_of_render_targets_minus_one = 1 - 1;
	TileRenderingModeCfgCommon->image_width_pixels = args->renderWidth;
	TileRenderingModeCfgCommon->image_height_pixels = args->renderHeight;
	TileRenderingModeCfgCommon->maximum_bpp_of_all_render_targets = V3D_INTERNAL_BPP_32;
	TileRenderingModeCfgCommon->multisample_mode_4x = FALSE;
	TileRenderingModeCfgCommon->double_buffer_in_non_ms_mode = FALSE;
	TileRenderingModeCfgCommon->early_z_test_and_update_direction = v3d_EARLY_Z_DIRECTION_LT_LE;
	TileRenderingModeCfgCommon->early_z_disable = FALSE;
	TileRenderingModeCfgCommon->internal_depth_type = V3D_INTERNAL_TYPE_DEPTH16;
	TileRenderingModeCfgCommon->early_depth_stencil_clear = FALSE;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(
		renderCommandList, v3d_OP_TILE_RENDERING_MODE_CFG_CLEAR_COLORS_PART1,
		v3d_tile_rendering_mode_cfg_clear_colors_part1, TileRenderingModeCfgClearColorsPart1);
	assert(TileRenderingModeCfgClearColorsPart1);
	TileRenderingModeCfgClearColorsPart1->sub_id = 3;
	TileRenderingModeCfgClearColorsPart1->render_target_number = 0;
	TileRenderingModeCfgClearColorsPart1->clear_color_low_32_bits = 0xFF101010;
	TileRenderingModeCfgClearColorsPart1->clear_color_next_24_bits = 0x000000;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_RENDERING_MODE_CFG_COLOR,
							   v3d_tile_rendering_mode_cfg_color, TileRenderingModeCfgColor);
	assert(TileRenderingModeCfgColor);
	TileRenderingModeCfgColor->sub_id = 1;
	TileRenderingModeCfgColor->render_target_0_internal_bpp = V3D_INTERNAL_BPP_32;
	TileRenderingModeCfgColor->render_target_0_internal_type = V3D_INTERNAL_TYPE_8;
	TileRenderingModeCfgColor->render_target_0_clamp = V3D_RENDER_TARGET_CLAMP_NONE;
	TileRenderingModeCfgColor->render_target_1_internal_bpp = V3D_INTERNAL_BPP_32;
	TileRenderingModeCfgColor->render_target_1_internal_type = V3D_INTERNAL_TYPE_8I;
	TileRenderingModeCfgColor->render_target_1_clamp = V3D_RENDER_TARGET_CLAMP_NONE;
	TileRenderingModeCfgColor->render_target_2_internal_bpp = V3D_INTERNAL_BPP_32;
	TileRenderingModeCfgColor->render_target_2_internal_type = V3D_INTERNAL_TYPE_8I;
	TileRenderingModeCfgColor->render_target_2_clamp = V3D_RENDER_TARGET_CLAMP_NONE;
	TileRenderingModeCfgColor->render_target_3_internal_bpp = V3D_INTERNAL_BPP_32;
	TileRenderingModeCfgColor->render_target_3_internal_type = V3D_INTERNAL_TYPE_8I;
	TileRenderingModeCfgColor->render_target_3_clamp = V3D_RENDER_TARGET_CLAMP_NONE;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	}

	// Must be last
	V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_RENDERING_MODE_CFG_ZS_CLEAR_VALUES,
	                           v3d_tile_rendering_mode_cfg_zs_clear_values, TileRenderingModeCfgZSClearValues);
	assert(TileRenderingModeCfgZSClearValues);
	TileRenderingModeCfgZSClearValues->sub_id = 2;
	TileRenderingModeCfgZSClearValues->stencil_clear_value = 0;
	TileRenderingModeCfgZSClearValues->z_clear_value = 1.f;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	}

	V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_LIST_INITIAL_BLOCK_SIZE,
							   v3d_tile_list_initial_block_size, TileListInitialBlockSize);
	assert(TileListInitialBlockSize);
	TileListInitialBlockSize->size_of_first_block_in_chained_tile_lists = 0;
	TileListInitialBlockSize->use_auto_chained_tile_lists = TRUE;
	if (compareBuffer)
	{
		assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	}

	u32 supertile_w = 0;
	u32 supertile_h = 0;
	u32 frame_w_supertile = 0;
	u32 frame_h_supertile = 0;
	getSuperTiles(&supertile_w, &supertile_h, &frame_w_supertile,
				   &frame_h_supertile, args->tilesX, args->tilesY);
	// Render layer
	{
		int layerIndex = 0;
		u32 tileAllocationOffset = layerIndex * args->tilesX * args->tilesY * 64;

		V3D_BUFFER_ALLOC_OPERATION(
			renderCommandList, v3d_OP_MULTICORE_RENDERING_TILE_LIST_SET_BASE,
			v3d_multicore_rendering_tile_list_set_base, MulticoreRenderingTileListSetBase);
		assert(MulticoreRenderingTileListSetBase);
		MulticoreRenderingTileListSetBase->tile_list_set_number = 0;
		MulticoreRenderingTileListSetBase->address_rshift_6 =
			(V3D_ARM_TO_BUS_ADDR(args->tileAllocation) + tileAllocationOffset) >> 6;
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_MULTICORE_RENDERING_SUPERTILE_CFG, v3d_multicore_rendering_supertile_cfg,
								   MulticoreRenderingSupertileCfg);
		assert(MulticoreRenderingSupertileCfg);
		MulticoreRenderingSupertileCfg->supertile_width_in_tiles_minus_one = supertile_w - 1;
		MulticoreRenderingSupertileCfg->supertile_height_in_tiles_minus_one = supertile_h - 1;
		MulticoreRenderingSupertileCfg->total_frame_width_in_supertiles = frame_w_supertile;
		MulticoreRenderingSupertileCfg->total_frame_height_in_supertiles = frame_h_supertile;
		MulticoreRenderingSupertileCfg->total_frame_width_in_tiles = args->tilesX;
		MulticoreRenderingSupertileCfg->total_frame_height_in_tiles = args->tilesY;
		MulticoreRenderingSupertileCfg->number_of_bin_tile_lists_minus_one = 1 - 1;
		MulticoreRenderingSupertileCfg->supertile_raster_order = FALSE;
		// (todo performance) Should this be enabled?
		MulticoreRenderingSupertileCfg->multicore_enable = FALSE;
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		// clear
		{
			V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_COORDINATES,
									   v3d_tile_coordinates, TileCoordinates);
			assert(TileCoordinates);
			TileCoordinates->tile_column_number = 0;
			TileCoordinates->tile_row_number = 0;
		}
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(renderCommandList, v3d_OP_END_OF_LOADS,
												v3d_end_of_loads);
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		{
			V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_STORE_TILE_BUFFER_GENERAL,
									   v3d_store_tile_buffer_general, StoreTileBufferGeneral);
			assert(StoreTileBufferGeneral);
			StoreTileBufferGeneral->buffer_to_store = v3d_NONE;
		}
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		{
			V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_CLEAR_TILE_BUFFERS,
									   v3d_clear_tile_buffers, ClearTileBuffers);
			assert(ClearTileBuffers);
			ClearTileBuffers->clear_all_render_targets = TRUE;
			ClearTileBuffers->clear_z_stencil_buffer = TRUE;
		}

		V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(renderCommandList, v3d_OP_END_OF_TILE_MARKER,
												v3d_end_of_tile_marker);
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		// dummy store: workaround race condition
		{
			V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_TILE_COORDINATES,
									   v3d_tile_coordinates, TileCoordinates);
			assert(TileCoordinates);
			TileCoordinates->tile_column_number = 0;
			TileCoordinates->tile_row_number = 0;
			if (compareBuffer)
			{
				assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
			}

			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(renderCommandList, v3d_OP_END_OF_LOADS,
													v3d_end_of_loads);
			if (compareBuffer)
			{
				assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
			}

			V3D_BUFFER_ALLOC_OPERATION(renderCommandList, v3d_OP_STORE_TILE_BUFFER_GENERAL,
									   v3d_store_tile_buffer_general, StoreTileBufferGeneral);
			assert(StoreTileBufferGeneral);
			StoreTileBufferGeneral->buffer_to_store = v3d_NONE;
			if (compareBuffer)
			{
				assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
			}

			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(
				renderCommandList, v3d_OP_END_OF_TILE_MARKER, v3d_end_of_tile_marker);
			if (compareBuffer)
			{
				assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
			}
		}

		V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(renderCommandList, v3d_OP_FLUSH_VCD_CACHE,
												v3d_flush_vcd_cache);
		if (compareBuffer)
		{
			assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		}

		// Indirect, used for generic tiles
		{
			indirectTileRenderCommandListStartCommands = V3D_BUFFER_WRITE_HEAD(*indirectTileRenderCommandList);
			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(indirectTileRenderCommandList,
													v3d_OP_TILE_COORDINATES_IMPLICIT,
													v3d_tile_coordinates_implicit);
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}
			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(indirectTileRenderCommandList,
													v3d_OP_END_OF_LOADS, v3d_end_of_loads);
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}

			V3D_BUFFER_ALLOC_OPERATION(indirectTileRenderCommandList, v3d_OP_PRIM_LIST_FORMAT,
									   v3d_prim_list_format, PrimListFormat);
			assert(PrimListFormat);
			PrimListFormat->primitive_type = v3d_LIST_TRIANGLES;
			PrimListFormat->tri_strip_or_fan = FALSE;
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}

			// SetInstanceId 0?

			V3D_BUFFER_ALLOC_OPERATION(indirectTileRenderCommandList, v3d_OP_BRANCH_TO_IMPLICIT_TILE_LIST,
									   v3d_branch_to_implicit_tile_list, BranchToImplicitTileList);
			assert(BranchToImplicitTileList);
			BranchToImplicitTileList->tile_list_set_number = 0;
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}

			// Raster to render target store
			// Note: this sets the render target address!
			{
				V3D_BUFFER_ALLOC_OPERATION(
					indirectTileRenderCommandList, v3d_OP_STORE_TILE_BUFFER_GENERAL,
					v3d_store_tile_buffer_general, StoreTileBufferGeneral);
				assert(StoreTileBufferGeneral);
				StoreTileBufferGeneral->buffer_to_store = v3d_RENDER_TARGET_0;
				StoreTileBufferGeneral->memory_format = V3D_MEMORY_FORMAT_RASTER;
				StoreTileBufferGeneral->flip_y = FALSE;
				StoreTileBufferGeneral->dither_mode = V3D_DITHER_MODE_NONE;
				StoreTileBufferGeneral->decimate_mode = V3D_DECIMATE_MODE_SAMPLE_0;
				StoreTileBufferGeneral->output_image_format = V3D_OUTPUT_IMAGE_FORMAT_RGBA8;
				StoreTileBufferGeneral->clear_buffer_being_stored = FALSE;
				StoreTileBufferGeneral->channel_reverse = FALSE;
				StoreTileBufferGeneral->r_b_swap = TRUE;
				// stride when in V3D_MEMORY_FORMAT_RASTER, maybe others too
				StoreTileBufferGeneral->height_in_ub_or_stride = args->renderTargetStride;
				StoreTileBufferGeneral->height = 0;
				StoreTileBufferGeneral->address = V3D_ARM_TO_BUS_ADDR(args->startRenderTarget);
				if (indirectCompareBuffer)
				{
					assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
				}
			}

			// Z buffer store
			{
				V3D_BUFFER_ALLOC_OPERATION(indirectTileRenderCommandList, v3d_OP_STORE_TILE_BUFFER_GENERAL,
										   v3d_store_tile_buffer_general, StoreTileBufferGeneral);
				assert(StoreTileBufferGeneral);
				StoreTileBufferGeneral->buffer_to_store = v3d_Z;
				StoreTileBufferGeneral->memory_format = V3D_MEMORY_FORMAT_UIF_XOR;
				StoreTileBufferGeneral->flip_y = FALSE;
				StoreTileBufferGeneral->dither_mode = V3D_DITHER_MODE_NONE;
				StoreTileBufferGeneral->decimate_mode = V3D_DECIMATE_MODE_SAMPLE_0;
				StoreTileBufferGeneral->output_image_format = V3D_OUTPUT_IMAGE_FORMAT_D16;
				StoreTileBufferGeneral->clear_buffer_being_stored = FALSE;
				StoreTileBufferGeneral->channel_reverse = FALSE;
				StoreTileBufferGeneral->r_b_swap = FALSE;
				StoreTileBufferGeneral->height_in_ub_or_stride = args->zBufferStride;
				StoreTileBufferGeneral->height = 0;
				StoreTileBufferGeneral->address = V3D_ARM_TO_BUS_ADDR(args->zBuffer);
				if (indirectCompareBuffer)
				{
					assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
				}
			}

			// old?
			V3D_BUFFER_ALLOC_OPERATION(indirectTileRenderCommandList, v3d_OP_CLEAR_TILE_BUFFERS,
									   v3d_clear_tile_buffers, ClearTileBuffers);
			assert(ClearTileBuffers);
			ClearTileBuffers->clear_all_render_targets = TRUE;
			ClearTileBuffers->clear_z_stencil_buffer = TRUE;
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}

			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(indirectTileRenderCommandList,
													v3d_OP_END_OF_TILE_MARKER,
													v3d_end_of_tile_marker);
			V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(indirectTileRenderCommandList,
													v3d_OP_RETURN_FROM_SUB_LIST,
													v3d_return_from_sub_list);
			if (indirectCompareBuffer)
			{
				assert(memcmp(indirectCompareBuffer, indirectTileRenderCommandList->start, indirectTileRenderCommandList->used) == 0);
			}
		}

		// Link the indirect into the render command list
		V3D_BUFFER_ALLOC_OPERATION(
			renderCommandList, v3d_OP_START_ADDRESS_OF_GENERIC_TILE_LIST,
			v3d_start_address_of_generic_tile_list, StartAddressOfGenericTileList);
		assert(StartAddressOfGenericTileList);
		if (compareBuffer && args->indirectStart && args->indirectEnd)
		{
			StartAddressOfGenericTileList->start = args->indirectStart;
			StartAddressOfGenericTileList->end = args->indirectEnd;
		}
		else
		{
			StartAddressOfGenericTileList->start = V3D_ARM_TO_BUS_ADDR(indirectTileRenderCommandListStartCommands);
			StartAddressOfGenericTileList->end =
				V3D_ARM_TO_BUS_ADDR(indirectTileRenderCommandList->start + indirectTileRenderCommandList->used);
		}
		// if (compareBuffer)
		// {
		// 	assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
		// }
	}

	// Add supertile coordinates
	{
		int max_supertile_x = (args->renderWidth - 1) / (args->tileWidth * supertile_w);
		int max_supertile_y = (args->renderHeight - 1) / (args->tileHeight * supertile_h);
		for (int y = 0; y <= max_supertile_y; y++)
		{
			for (int x = 0; x <= max_supertile_x; x++)
			{
				V3D_BUFFER_ALLOC_OPERATION(
					renderCommandList, v3d_OP_SUPERTILE_COORDINATES,
					v3d_supertile_coordinates, SuperTileCoordinates);
				assert(SuperTileCoordinates);
				SuperTileCoordinates->column_number_in_supertiles = x;
				SuperTileCoordinates->row_number_in_supertiles = y;
				// if (compareBuffer)
				// {
				// 	assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
				// }
			}
		}
	}

	V3D_BUFFER_QUEUE_OPERATION_NO_ARGUMENTS(renderCommandList, v3d_OP_END_OF_RENDERING,
											v3d_end_of_rendering);
	// if (compareBuffer)
	// {
	// 	assert(memcmp(compareBuffer, renderCommandList->start, renderCommandList->used) == 0);
	// }

	if (invalidateCacheStart)
	{
		*invalidateCacheStart = indirectTileRenderCommandList->start;
		*invalidateCacheSize = indirectTileRenderCommandList->used;
	}
}

u8* compareRenderCommandList(PrepareRenderCommandListArguments* args,
							 u8* compareBuffer, u8* indirectCompareBuffer, int* renderCommandListSizeOut)
{
	static u8 __attribute__((aligned(64))) renderCommandListBuffer[4096] = {0};
	v3d_static_buffer renderCommandList = {renderCommandListBuffer, 0,
	                                        sizeof(renderCommandListBuffer)};
	static u8 __attribute__((aligned(64))) indirectTileRenderCommandListBuffer[4096] = {0};
	v3d_static_buffer indirectTileRenderCommandList = {
	    indirectTileRenderCommandListBuffer, 0, sizeof(indirectTileRenderCommandListBuffer)};
	prepareRenderCommandList(&renderCommandList, &indirectTileRenderCommandList, args,
							 compareBuffer, indirectCompareBuffer, 0, 0);
	*renderCommandListSizeOut = renderCommandList.used;
	return renderCommandList.start;
}

#define V3D_CTL_INT_STS                                0x00050
#define V3D_CTL_INT_SET                                0x00054
#define V3D_CTL_INT_CLR                                0x00058
#define V3D_CTL_INT_MSK_STS                            0x0005c
#define V3D_CTL_INT_MSK_SET                            0x00060
#define V3D_CTL_INT_MSK_CLR                            0x00064
# define V3D_INT_QPU_MASK                              V3D_MASK(27, 16)
# define V3D_INT_QPU_SHIFT                             16
# define V3D_INT_CSDDONE                               BIT(7)
# define V3D_V7_INT_CSDDONE                            BIT(6)
# define V3D_INT_PCTR                                  BIT(6)
# define V3D_V7_INT_PCTR                               BIT(5)
# define V3D_INT_GMPV                                  BIT(5)
# define V3D_INT_TRFB                                  BIT(4)
# define V3D_INT_SPILLUSE                              BIT(3)
# define V3D_INT_OUTOMEM                               BIT(2)
# define V3D_INT_FLDONE                                BIT(1)
# define V3D_INT_FRDONE                                BIT(0)

#define V3D_HUB_INT_STS                                0x00050
#define V3D_HUB_INT_SET                                0x00054
#define V3D_HUB_INT_CLR                                0x00058
#define V3D_HUB_INT_MSK_STS                            0x0005c
#define V3D_HUB_INT_MSK_SET                            0x00060
#define V3D_HUB_INT_MSK_CLR                            0x00064
# define V3D_V7_HUB_INT_GMPV                           BIT(6)
# define V3D_HUB_INT_MMU_WRV                           BIT(5)
# define V3D_HUB_INT_MMU_PTI                           BIT(4)
# define V3D_HUB_INT_MMU_CAP                           BIT(3)
# define V3D_HUB_INT_MSO                               BIT(2)
# define V3D_HUB_INT_TFUC                              BIT(1)
# define V3D_HUB_INT_TFUF                              BIT(0)

#define V3D_CORE_IRQS(ver) ((u32)(V3D_INT_OUTOMEM |	\
				  V3D_INT_FLDONE |	\
				  V3D_INT_FRDONE |	\
				  (ver < 71 ? V3D_INT_CSDDONE : V3D_V7_INT_CSDDONE) |	\
				  (ver < 71 ? V3D_INT_GMPV : 0)))

#define V3D_HUB_IRQS(ver) ((u32)(V3D_HUB_INT_MMU_WRV |	\
				 V3D_HUB_INT_MMU_PTI |	\
				 V3D_HUB_INT_MMU_CAP |	\
				 V3D_HUB_INT_TFUC |		\
				 (ver >= 71 ? V3D_V7_HUB_INT_GMPV : 0)))

#include "circle/interrupt.h"

// Primitive tile binning binning pool overflow address
#define V3D_PTB_BPOA                                   0x00308
// Primitive tile binning binning pool overflow size
#define V3D_PTB_BPOS                                   0x0030c

// See linux/drivers/gpu/drm/v3d/v3d_irq.c
static void v3d_handle_interrupt(void *pParam)
{
	u32 interruptStatus = read32(V3D_CORE0 + V3D_CTL_INT_STS);
	// Acknowledge handling of this interrupt
	write32(V3D_CORE0 + V3D_CTL_INT_CLR, interruptStatus);
	if (interruptStatus & V3D_INT_OUTOMEM)
	{
		/*Note that the OOM status is edge signaled, so the
		 * interrupt won't happen again until the we actually
		 * add more memory.  Also, as of V3D 4.1, FLDONE won't
		 * be reported until any OOM state has been cleared.*/
		// Give the binner overflow memory
		// Note that "the HW signals OOM before it's fully OOM, so the binner might just barely complete."
		// See linux/drivers/gpu/drm/v3d/v3d_irq.c
		// This matches the kernel hard-coded value
		int overflowMemorySize = 256 * 1024;
		// NOTE: 4096 aligned, which matches in the kernel V3D_MMU_PAGE_SHIFT
		void* overflowMemory = Static_HeapAllocate(overflowMemorySize, HEAP_LOW);
		write32(V3D_CORE0 + V3D_PTB_BPOA, V3D_ARM_TO_BUS_ADDR(overflowMemory));
		write32(V3D_CORE0 + V3D_PTB_BPOS, overflowMemorySize);
		/* assert(0 && "V3D out of memory"); */
	}

	/* V3D 4.2 wires the hub and core IRQs together, so if we &
	 * didn't see the common one then check hub for MMU IRQs.*/
	{
		u32 hubInterruptStatus = read32(V3D_HUB_BASE + V3D_HUB_INT_STS);
		// Acknowledge handling of this interrupt
		write32(V3D_HUB_BASE + V3D_HUB_INT_CLR, hubInterruptStatus);
	}
}

void v3d_enable_irqs(void)
{
	PeripheralEntry();

	// Clear any pending interrupts
	write32(V3D_CORE0 + V3D_CTL_INT_CLR, V3D_CORE_IRQS(42));

	write32(V3D_HUB_BASE + V3D_HUB_INT_CLR, V3D_HUB_IRQS(42));

	write32(V3D_CORE0 + V3D_CTL_INT_MSK_SET, ~V3D_CORE_IRQS(42));
	write32(V3D_CORE0 + V3D_CTL_INT_MSK_CLR, V3D_CORE_IRQS(42));

	write32(V3D_HUB_BASE + V3D_HUB_INT_MSK_SET, ~V3D_HUB_IRQS(42));
	write32(V3D_HUB_BASE + V3D_HUB_INT_MSK_CLR, V3D_HUB_IRQS(42));

	PeripheralExit();

	// TODO I'm not sure why this doesn't work
	//assert(ARM_IRQ_V3D == MachineInfo_GetV3DInterrupt() && "V3D Interrupt doesn't match device tree");
	InterruptSystem_ConnectIRQ (ARM_IRQ_V3D, v3d_handle_interrupt, 0);
}

#define V3D_HUB_IDENT1                                 0x0000c
# define V3D_HUB_IDENT1_WITH_MSO                       BIT(19)
# define V3D_HUB_IDENT1_WITH_TSY                       BIT(18)
# define V3D_HUB_IDENT1_WITH_TFU                       BIT(17)
# define V3D_HUB_IDENT1_WITH_L3C                       BIT(16)
# define V3D_HUB_IDENT1_NHOSTS_MASK                    15 //V3D_MASK(15, 12)
# define V3D_HUB_IDENT1_NHOSTS_SHIFT                   12
# define V3D_HUB_IDENT1_NCORES_MASK                    15 // V3D_MASK(11, 8)
# define V3D_HUB_IDENT1_NCORES_SHIFT                   8
# define V3D_HUB_IDENT1_REV_MASK                       15 //V3D_MASK(7, 4)
# define V3D_HUB_IDENT1_REV_SHIFT                      4
# define V3D_HUB_IDENT1_TVER_MASK                      15 // V3D_MASK(3, 0)
# define V3D_HUB_IDENT1_TVER_SHIFT                     0
# define V3D_HUB_IDENT1_VPM_SIZE_MASK                      15
# define V3D_HUB_IDENT1_VPM_SIZE_SHIFT                     28

// See Mesa/src/broadcom/common/v3d_device_info.c
void v3d_read_info(struct v3d_device_info* deviceInfo)
{
	u32 identity1 = read32(V3D_HUB_BASE + V3D_HUB_IDENT1);
	u32 version = (((identity1 >> V3D_HUB_IDENT1_TVER_SHIFT) & V3D_HUB_IDENT1_TVER_MASK) * 10) +
	              ((identity1 >> V3D_HUB_IDENT1_REV_SHIFT) & V3D_HUB_IDENT1_REV_MASK);
	deviceInfo->ver = version;
	// TODO Not correct
	/* deviceInfo->vpm_size = */
	/*     (identity1 >> V3D_HUB_IDENT1_VPM_SIZE_SHIFT & V3D_HUB_IDENT1_VPM_SIZE_MASK) * 8192; */
	/* int numSlices = (identity1 >> 4) & 0xf; */
	/* int numQpus = (identity1 >> 8) & 0xf; */
	/* deviceInfo->qpu_count = numSlices * numQpus; */
	deviceInfo->has_accumulators = version < 71;
}

typedef struct { u64 state;  u64 inc; } pcg32_random_t;

u32 pcg32_random_r(pcg32_random_t* rng)
{
    u64 oldstate = rng->state;
    // Advance internal state
    rng->state = oldstate * 6364136223846793005ULL + (rng->inc|1);
    // Calculate output function (XSH RR), uses old state for max ILP
    u32 xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
    u32 rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

static pcg32_random_t s_random;

#define NUM_TRIANGLES_SQR 316

static float g_pos_attr_buff[3 * 3];

static u8 g_color_attr_buff[NUM_TRIANGLES * 4];

void prepareVertices(int renderWidth, int renderHeight)
{
	int trianglesPerAxis = NUM_TRIANGLES_SQR;  // todo get sqrt
	/* int trianglesPerAxis = 100;  // todo get sqrt */
	float triangleWidth = renderWidth / (float)trianglesPerAxis;
	float triangleHeight = renderHeight / (float)trianglesPerAxis;
	// float triangleVary = (triangleWidth / (float)renderWidth);
	float addX[] = {0.f, triangleWidth, 0.f};
	float addY[] = {0.f, 0.f, triangleHeight};
	float initialOffsetX = -1.f;
	float initialOffsetY = -1.f;
	float constantOffsetX = 0.0f;
	float constantOffsetY = 0.01f;
	for (int triangleIndex = 0; triangleIndex < NUM_TRIANGLES; ++triangleIndex)
	{
		for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex)
		{
			// vertex[0] = ((pcg32_random_r(&s_random) / (float)4294967295) * 2.f) - 1.f;
			// vertex[1] = ((pcg32_random_r(&s_random) / (float)4294967295) * 2.f) - 1.f;
			// vertex[2] = ((pcg32_random_r(&s_random) / (float)4294967295) * 2.f) - 1.f;
			int triangleX = triangleIndex % trianglesPerAxis;
			int triangleY = triangleIndex / trianglesPerAxis;
			float triangleXPixels = ((triangleX * triangleWidth) + addX[vertexIndex]);
			float triangleYPixels = ((triangleY * triangleHeight) + addY[vertexIndex]);
			float triangleXScreen = (((triangleXPixels / renderWidth) * 2.f) - 1.f);
			float triangleYScreen = (((triangleYPixels / renderHeight) * 2.f) - 1.f);
			if (!vertexIndex)
			{
				g_instance_offsets[(triangleIndex * 2) + 0] = triangleXScreen - initialOffsetX + constantOffsetX;
				g_instance_offsets[(triangleIndex * 2) + 1] = triangleYScreen - initialOffsetY + constantOffsetY;
				u8* color = &g_color_attr_buff[(triangleIndex * 4)];
				color[0] = pcg32_random_r(&s_random) % 256;
				color[1] = pcg32_random_r(&s_random) % 256;
				color[2] = pcg32_random_r(&s_random) % 256;
				color[3] = 0xff;
			}
			if (triangleIndex == 0)
			{
				float* vertex = &g_pos_attr_buff[(vertexIndex * 3)];
				vertex[0] = triangleXScreen;
				vertex[1] = triangleYScreen + constantOffsetY;
				vertex[2] = 0.f;
			}
		}
	}
}

TShutdownMode Kernel_Run (void)
{
	boolean bVSync = TRUE;
	C2DGraphics_Create(&m_2DGraphics, KernelOptions_GetWidth(), KernelOptions_GetHeight(), bVSync, 0);

	if (!C2DGraphics_Initialize(&m_2DGraphics))
	{
		assert(0 && "Failed to get framebuffer");
	}

	if (!InterruptSystem_Initialize())
	{
		assert(0 && "Failed to initialize interrupt system");
	}

	v3d_power_on();
	struct v3d_device_info deviceInfo = {0};
	v3d_read_info(&deviceInfo);
	v3d_enable_irqs();

	debugRegisters();

	C2DGraphics_DrawText(&m_2DGraphics, 10, 100, 0xffffff, "Hello, V3D! 1", AlignLeft);
	C2DGraphics_UpdateDisplay(&m_2DGraphics);
	C2DGraphics_DrawText(&m_2DGraphics, 10, 100, 0xffffff, "Hello, V3D! 0", AlignLeft);
	C2DGraphics_UpdateDisplay(&m_2DGraphics);

	// Assemble shaders
	typedef struct shader_assemble
	{
		const char* name;
		const char** assemblyLines;
		int numAssemblyLines;
		struct v3d_qpu_instr* unpackedInstructions;
		v3d_qpu_instruction* instructions;
		int maxNumInstructions;
		int* numInstructions;
	} shader_assemble;
	shader_assemble shadersToAssemble[] = {
	    {"Fragment shader", g_fragment_shader_assembly, ArraySize(g_fragment_shader_assembly),
	     g_fragment_shader_unpacked_instructions, g_fragment_shader_buffer,
	     ArraySize(g_fragment_shader_buffer), &g_fragment_shader_buffer_num_instructions},
	    {"Vertex shader", g_vertex_shader_assembly, ArraySize(g_vertex_shader_assembly),
	     g_vertex_shader_unpacked_instructions, g_vertex_shader_buffer,
	     ArraySize(g_vertex_shader_buffer), &g_vertex_shader_buffer_num_instructions},
	    {"Coordinate shader", g_coordinate_shader_assembly, ArraySize(g_coordinate_shader_assembly),
	     g_coordinate_shader_unpacked_instructions, g_coordinate_shader_buffer,
	     ArraySize(g_coordinate_shader_buffer), &g_coordinate_shader_buffer_num_instructions},
	};
	for (int shaderIndex = 0; shaderIndex < ArraySize(shadersToAssemble); ++shaderIndex)
	{
		shader_assemble* shader = &shadersToAssemble[shaderIndex];
		for (int assemblyLine = 0; assemblyLine < shader->numAssemblyLines; ++assemblyLine)
		{
			struct v3d_qpu_assemble_arguments assembleArguments = {0};
			assembleArguments.devinfo = deviceInfo;
			assembleArguments.assembly = shader->assemblyLines[assemblyLine];
			int numCharactersRead = v3d_qpu_assemble(&assembleArguments);
			if (!numCharactersRead)
			{
				char assembleResults[512] = {0};
				SString_Format(
				    assembleResults, sizeof(assembleResults),
				    "Failed to assemble %s instruction [%d] column %d with error:\n%s\n'%s'",
				    shader->name, assemblyLine, assembleArguments.errorAtOffset,
				    assembleArguments.errorMessage, assembleArguments.assembly);
				C2DGraphics_DrawTextMultiline(&m_2DGraphics, 10, 200, 0xffaaaa, assembleResults);
				C2DGraphics_UpdateDisplay(&m_2DGraphics);
				return ShutdownHalt;
			}
			shader->unpackedInstructions[*shader->numInstructions] = assembleArguments.instruction;
			if (!v3d_qpu_instr_pack(&assembleArguments.devinfo, &assembleArguments.instruction,
			                        &shader->instructions[*shader->numInstructions]))
			{
				char assembleResults[512] = {0};
				SString_Format(assembleResults, sizeof(assembleResults),
				               "Failed to pack %s instruction [%d]\n'%s'", shader->name,
				               assemblyLine, assembleArguments.assembly);
				C2DGraphics_DrawTextMultiline(&m_2DGraphics, 10, 200, 0xffaaaa, assembleResults);
				C2DGraphics_UpdateDisplay(&m_2DGraphics);
				return ShutdownHalt;
			}
			++(*shader->numInstructions);
			assert((*shader->numInstructions) < shader->maxNumInstructions &&
			       "Ran out of space for shader instructions");
		}

		struct v3d_qpu_validate_result validateResults = {0};
		if (!v3d_qpu_validate(&deviceInfo, shader->unpackedInstructions, *shader->numInstructions,
		                      &validateResults))
		{
			char assembleResults[512] = {0};
			SString_Format(assembleResults, sizeof(assembleResults),
			               "Validation error in %s at instruction [%d]\n%s\n'%s'", shader->name,
			               validateResults.errorInstructionIndex, validateResults.errorMessage,
			               shader->assemblyLines[validateResults.errorInstructionIndex]);
			C2DGraphics_DrawTextMultiline(&m_2DGraphics, 10, 200, 0xffaaaa, assembleResults);
			C2DGraphics_UpdateDisplay(&m_2DGraphics);
			return ShutdownHalt;
		}
	}

	// Set up render target

	// Offset into render target (the GPU could do this for us if we want)
	int targetStartX = 200;
	int targetStartY = 0;
	int renderWidth = C2DGraphics_GetWidth(&m_2DGraphics) - targetStartX;
	int renderHeight = C2DGraphics_GetHeight(&m_2DGraphics) - targetStartY;

	// Z buffer memory
	void* zBuffer = Static_HeapAllocate(renderWidth * renderHeight * 2, HEAP_LOW);
	assert(zBuffer);
	// (todo understanding) Why this math?
	// int zBufferStride = renderWidth * 2; // 16-bit Z buffer //renderHeight + 15 / (2 * 4);
	int zBufferStride = renderHeight + 15 / (2 * 4);

	prepareVertices(renderWidth, renderHeight);

	// int numTriangles = ArraySize(g_vertex_positions) / (3 * 3); // 3 floats per vertex, 3 vertices per triangle
	// int vertexCount = numTriangles * 3;
	int numTriangles = NUM_TRIANGLES;
	int vertexCount = numTriangles * 3;

	// decreases if msaa/double buffering/multiple color attachement/more bpp
	int tileWidth = 64;
	int tileHeight = 64;
	int tilesX = divideRoundUp(renderWidth, tileWidth);
    int tilesY = divideRoundUp(renderHeight, tileHeight);

	//
	// Set up the binning command list
	//
	static u8 __attribute__((aligned(64))) binningCommandListBuffer[4096] = {0};
	v3d_static_buffer binningCommandList = {binningCommandListBuffer, 0,
	                                        sizeof(binningCommandListBuffer)};
	int numAttributes = 0;
	u8* invalidateShaderStateCacheStart = 0;
	u64 invalidateShaderStateCacheSize = 0;
	PrepareShaderStateRecordArguments prepareShaderStateRecordArgs = {0};
	prepareShaderStateRecordArgs.renderWidth = renderWidth;
	prepareShaderStateRecordArgs.renderHeight = renderHeight;
	prepareShaderStateRecordArgs.fragment_shader_buffer = (u8*)g_fragment_shader_buffer;
	prepareShaderStateRecordArgs.fragment_shader_buffer_size =
	    g_fragment_shader_buffer_num_instructions * sizeof(g_fragment_shader_buffer[0]);
	prepareShaderStateRecordArgs.vertex_shader_buffer = (u8*)g_vertex_shader_buffer;
	prepareShaderStateRecordArgs.vertex_shader_buffer_size =
	    g_vertex_shader_buffer_num_instructions * sizeof(g_vertex_shader_buffer[0]);
	prepareShaderStateRecordArgs.coordinate_shader_buffer = (u8*)g_coordinate_shader_buffer;
	prepareShaderStateRecordArgs.coordinate_shader_buffer_size =
	    g_coordinate_shader_buffer_num_instructions * sizeof(g_coordinate_shader_buffer[0]);
	prepareShaderStateRecordArgs.vertex_positions = (u8*)g_pos_attr_buff;
	prepareShaderStateRecordArgs.vertex_colors_rgba = (u8*)g_color_attr_buff;
	// prepareShaderStateRecordArgs.vertex_positions = (u8*)g_vertex_positions;
	// prepareShaderStateRecordArgs.vertex_colors_rgba = (u8*)g_vertex_colors_rgba;
	v3d_gl_shader_state_record* shaderStateRecord =
	    prepareShaderStateRecord(&prepareShaderStateRecordArgs, &numAttributes,
	                             &invalidateShaderStateCacheStart, &invalidateShaderStateCacheSize);
	prepareBinningCommandList(&binningCommandList, renderWidth, renderHeight, numAttributes,
	                          shaderStateRecord, vertexCount, 0);

	GetTileStatesArguments tileStatesArgs = {0};
	getTileStates(&tileStatesArgs, renderWidth, renderHeight,
				  sizeof(g_vertex_positions), sizeof(g_vertex_colors_rgba));

	//
	// Set up the render command list
	//
	int renderTargetStride = 0;
	boolean useBufferA = TRUE;
	u8* startRenderTargetA = 0;
	u8* startRenderTargetB = 0;
	{
		startRenderTargetA = (u8*)C2DGraphics_GetBuffer(&m_2DGraphics);
		int frameBufferStride = C2DGraphics_GetWidth(&m_2DGraphics) * sizeof(TScreenColor);
		startRenderTargetA += ((targetStartY * frameBufferStride) + (targetStartX * sizeof(TScreenColor)));
		renderTargetStride = frameBufferStride;
	}
	{
		// TODO This is wait for vsync, so slow. Just add api for double buffering
		C2DGraphics_UpdateDisplay(&m_2DGraphics);
		startRenderTargetB = (u8*)C2DGraphics_GetBuffer(&m_2DGraphics);
		int frameBufferStride = C2DGraphics_GetWidth(&m_2DGraphics) * sizeof(TScreenColor);
		startRenderTargetB += ((targetStartY * frameBufferStride) + (targetStartX * sizeof(TScreenColor)));
		useBufferA = FALSE;
	}

	static u8 __attribute__((aligned(64))) renderCommandListBuffer[4096] = {0};
	v3d_static_buffer renderCommandList = {renderCommandListBuffer, 0,
	                                       sizeof(renderCommandListBuffer)};
	static u8 __attribute__((aligned(64))) indirectTileRenderCommandListBufferA[4096] = {0};
	v3d_static_buffer indirectTileRenderCommandListA = {
	    indirectTileRenderCommandListBufferA, 0, sizeof(indirectTileRenderCommandListBufferA)};
	u8* invalidateIndirectCommandListCacheStart = 0;
	u64 invalidateIndirectCommandListCacheSize = 0;
	{
		PrepareRenderCommandListArguments renderCommandListArguments = {0};
		renderCommandListArguments.renderWidth = renderWidth;
		renderCommandListArguments.renderHeight = renderHeight;
		renderCommandListArguments.tilesX = tilesX;
		renderCommandListArguments.tilesY = tilesY;
		renderCommandListArguments.tileWidth = tileWidth;
		renderCommandListArguments.tileHeight = tileHeight;
		renderCommandListArguments.renderTargetStride = renderTargetStride;
		renderCommandListArguments.startRenderTarget = startRenderTargetA;
		renderCommandListArguments.zBufferStride = zBufferStride;
		renderCommandListArguments.zBuffer = (u8*)zBuffer;
		renderCommandListArguments.tileAllocation = tileStatesArgs.tileAllocation;
		prepareRenderCommandList(
		    &renderCommandList, &indirectTileRenderCommandListA, &renderCommandListArguments, 0, 0,
		    &invalidateIndirectCommandListCacheStart, &invalidateIndirectCommandListCacheSize);
	}

	static u8 __attribute__((aligned(64))) renderCommandListBufferB[4096] = {0};
	v3d_static_buffer renderCommandListB = {renderCommandListBufferB, 0,
		sizeof(renderCommandListBufferB)};
	static u8 __attribute__((aligned(64))) indirectTileRenderCommandListBufferB[4096] = {0};
	v3d_static_buffer indirectTileRenderCommandListB = {
	    indirectTileRenderCommandListBufferB, 0, sizeof(indirectTileRenderCommandListBufferB)};
	u8* invalidateIndirectCommandListCacheStartB = 0;
	u64 invalidateIndirectCommandListCacheSizeB = 0;
	{
		PrepareRenderCommandListArguments renderCommandListArguments = {0};
		renderCommandListArguments.renderWidth = renderWidth;
		renderCommandListArguments.renderHeight = renderHeight;
		renderCommandListArguments.tilesX = tilesX;
		renderCommandListArguments.tilesY = tilesY;
		renderCommandListArguments.tileWidth = tileWidth;
		renderCommandListArguments.tileHeight = tileHeight;
		renderCommandListArguments.renderTargetStride = renderTargetStride;
		renderCommandListArguments.startRenderTarget = startRenderTargetB;
		renderCommandListArguments.zBufferStride = zBufferStride;
		renderCommandListArguments.zBuffer = (u8*)zBuffer;
		renderCommandListArguments.tileAllocation = tileStatesArgs.tileAllocation;
		prepareRenderCommandList(&renderCommandListB, &indirectTileRenderCommandListB, &renderCommandListArguments, 0, 0,
								 &invalidateIndirectCommandListCacheStartB,
								 &invalidateIndirectCommandListCacheSizeB);
	}

	assert(!v3d_buffer_out_of_memory(&binningCommandList));
	assert(!v3d_buffer_out_of_memory(&renderCommandList));

	v3d_invalidate_caches();

	debugRegisters();

	// Since Circle sets up the static/executable memory as Normal, Inner shareable, we
	// must make sure that main memory has our command list, not just the processor cache,
	// so that the GPU can read it.
	// Note that we don't bother cleaning the tileStatesArgs because they are just memory
	// for the GPU; we didn't write anything to them.
	CleanDataCacheRange((u64)binningCommandList.start, binningCommandList.used);
	CleanDataCacheRange((u64)invalidateShaderStateCacheStart, invalidateShaderStateCacheSize);
	CleanDataCacheRange((u64)g_pos_attr_buff, sizeof(g_pos_attr_buff));
	CleanDataCacheRange((u64)g_color_attr_buff, sizeof(g_color_attr_buff));
	CleanDataCacheRange((u64)g_instance_offsets, sizeof(g_instance_offsets));

	CleanDataCacheRange((u64)renderCommandList.start, renderCommandList.used);
	CleanDataCacheRange((u64)renderCommandListB.start, renderCommandListB.used);
	CleanDataCacheRange((u64)invalidateIndirectCommandListCacheStart,
						invalidateIndirectCommandListCacheSize);
	CleanDataCacheRange((u64)invalidateIndirectCommandListCacheStartB,
						invalidateIndirectCommandListCacheSizeB);
	unsigned frameTimings[1000] = {0};
	int numFrames = ArraySize(frameTimings);
	for (int frameIndex = 0; frameIndex < numFrames; ++frameIndex)
	{
		// C2DGraphics_ClearScreen(&m_2DGraphics, BLACK_COLOR);
		unsigned startFrameMegaHZTicks = Timer_GetClockTicks();

		// Bin
		{
			u8 lastBinningFlush = v3d_get_binning_flush_count();
			/* Clear out the overflow allocation, so we don't
			 * reuse the overflow attached to a previous job.*/
			write32(V3D_CORE0 + V3D_PTB_BPOS, 0);
			v3d_start_binning_commands(
			    V3D_ARM_TO_BUS_ADDR(binningCommandList.start),
			    V3D_ARM_TO_BUS_ADDR(binningCommandList.start) + binningCommandList.used,
			    V3D_ARM_TO_BUS_ADDR(tileStatesArgs.tileAllocation),
			    tileStatesArgs.tileAllocationSize, V3D_ARM_TO_BUS_ADDR(tileStatesArgs.tileState));

			debugRegisters();

			v3d_wait_for_binning_flush(lastBinningFlush);

			// For debugging only; this is unnecessary for rendering
			/* CleanDataCacheRange((u64)tileStatesArgs.tileAllocation, */
			/* 					tileStatesArgs.tileAllocationSize); */
			/* CleanDataCacheRange((u64)tileStatesArgs.tileState, */
			/* 					tileStatesArgs.tileStateSize); */
		}

		// Render
		{
			u8 lastRenderFrame = v3d_get_render_frame_count();
			v3d_start_render_commands(
			    useBufferA ? V3D_ARM_TO_BUS_ADDR(renderCommandList.start) :
			                 V3D_ARM_TO_BUS_ADDR(renderCommandListB.start),
			    useBufferA ?
			        V3D_ARM_TO_BUS_ADDR(renderCommandList.start) + renderCommandList.used :
			        V3D_ARM_TO_BUS_ADDR(renderCommandListB.start) + renderCommandListB.used);

			v3d_wait_for_render_frame(lastRenderFrame);
		}

		/* g_instance_offsets[1] += 0.001f; */
		/* CleanDataCacheRange((u64)g_instance_offsets, sizeof(g_instance_offsets)); */

		unsigned endFrameMegaHZTicks = Timer_GetClockTicks();
		// Ignore timings during wrap-around
		if (endFrameMegaHZTicks > startFrameMegaHZTicks)
			frameTimings[frameIndex] = endFrameMegaHZTicks - startFrameMegaHZTicks;

		C2DGraphics_UpdateDisplay(&m_2DGraphics);
		useBufferA = !useBufferA;
	}

	const unsigned clockFrequencyHz = 1000000;
	float maxTimeMs = 0.f;
	float minTimeMs = 100.f;
	float meanTimeMs = 0.f;
	int numValidSamples = 0;
	for (unsigned int sampleIndex = 0; sampleIndex < ArraySize(frameTimings); ++sampleIndex)
	{
		unsigned sample = frameTimings[sampleIndex];
		if (!sample)
			continue;
		float sampleMs = (float)sample / (float)clockFrequencyHz;
		if (sampleMs < minTimeMs)
			minTimeMs = sampleMs;
		if (sampleMs > maxTimeMs)
			maxTimeMs = sampleMs;
		meanTimeMs += sampleMs;
		++numValidSamples;
	}
	if (numValidSamples)
		meanTimeMs /= numValidSamples;
	char timingResults[512] = {0};
	SString_Format(timingResults, sizeof(timingResults),
				   "Num samples: %d\n"
				   "Longest  frame: %f ms\n"
				   "Shortest frame: %f ms\n"
				   "Average  frame: %f ms\n"
				   "Average  frame: %f fps\n",
				   numValidSamples,
				   maxTimeMs,
				   minTimeMs,
				   meanTimeMs,
				   1.f / (meanTimeMs ? meanTimeMs : 1.f));
	C2DGraphics_DrawTextMultiline(&m_2DGraphics, 10, 200, 0xffffff, timingResults);
	C2DGraphics_UpdateDisplay(&m_2DGraphics);
	return ShutdownHalt;
}
