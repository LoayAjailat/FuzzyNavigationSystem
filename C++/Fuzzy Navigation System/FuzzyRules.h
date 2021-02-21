#pragma once
enum Velocity {
	Slow, Medium, Fast
};

int CC, CI, CF, IC, II, IF, FC, FI, FF,
	CCC, CCI, CCF, CIC, CII, CIF, CFC, CFI, CFF,
	ICC, ICI, ICF, IIC, III, IIF, IFC, IFI, IFF,
	FCC, FCI, FCF, FIC, FII, FIF, FFC, FFI, FFF;

/// Define the rules for Right Edge
// Left wheel
int LCC = Slow, LCI = Slow, LCF = Slow,
	LIC = Slow, LII = Slow, LIF = Slow,
	LFC = Fast, LFI = Medium, LFF = Medium;
int RE_L[9] = { LCC, LCI, LCF, LIC, LII, LIF, LFC, LFI, LFF };
// Right wheel
int RCC = Fast, RCI = Fast, RCF = Fast,
	RIC = Medium, RII = Slow, RIF = Medium,
	RFC = Medium, RFI = Slow, RFF = Slow;
int RE_R[9] = { RCC, RCI, RCF, RIC, RII, RIF, RFC, RFI, RFF };

/// Define the rules for Obstacle Avoidance
// Left wheel
int LCCC = 0, LCCI = Slow, LCCF = Slow,
	LCIC = Medium, LCII = Slow, LCIF = Slow,
	LCFC = Medium, LCFI = Slow, LCFF = Slow,
	LICC = Medium, LICI = Slow, LICF = Slow,
	LIIC = Fast, LIII = Slow, LIIF = Slow,
	LIFC = Fast, LIFI = Medium, LIFF = Slow,
	LFCC = Fast, LFCI = Fast, LFCF = Slow,
	LFIC = Fast, LFII = Fast, LFIF = Slow,
	LFFC = Fast, LFFI = Fast, LFFF = Medium;
int OA_L[27] = { LCCC, LCCI, LCCF, LCIC, LCII, LCIF, LCFC, LCFI, LCFF,
				LICC, LICI, LICF, LIIC, LIII, LIIF, LIFC, LIFI, LIFF,
				LFCC, LFCI, LFCF, LFIC, LFII, LFIF, LFFC, LFFI, LFFF };

// Right wheel
int RCCC = Fast, RCCI = Fast, RCCF = Fast,
	RCIC = Medium, RCII = Fast, RCIF = Fast,
	RCFC = Medium, RCFI = Fast, RCFF = Fast,
	RICC = Slow, RICI = Fast, RICF = Fast,
	RIIC = Slow, RIII = Slow, RIIF = Fast,
	RIFC = Slow, RIFI = Medium, RIFF = Fast,
	RFCC = Slow, RFCI = Slow, RFCF = Fast,
	RFIC = Slow, RFII = Medium, RFIF = Fast,
	RFFC = Slow, RFFI = Slow, RFFF = Medium;
int OA_R[27] = { RCCC, RCCI, RCCF, RCIC, RCII, RCIF, RCFC, RCFI, RCFF,
				RICC, RICI, RICF, RIIC, RIII, RIIF, RIFC, RIFI, RIFF,
				RFCC, RFCI, RFCF, RFIC, RFII, RFIF, RFFC, RFFI, RFFF };