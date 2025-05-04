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
