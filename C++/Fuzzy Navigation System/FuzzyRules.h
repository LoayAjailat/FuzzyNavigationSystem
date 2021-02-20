#pragma once
enum Velocity {
	Slow, Medium, Fast
};

int CC, CI, CF, IC, II, IF, FC, FI, FF;
int CCC, CCI, CCF, CIC, CII, CIF, CFC, CFI, CFF;
int ICC, ICI, ICF, IIC, III, IIF, IFC, IFI, IFF;
int FCC, FCI, FCF, FIC, FII, FIF, FFC, FFI, FFF;

/////// HOW CAN I GROUP THE RULES FOR EACH FUNCTION (RE,OA,GS)?
/// Define the rules for Right Edge
// Left wheel
int LCC = Slow, LCI = Slow, LCF = Slow;
int LIC = Slow, LII = Slow, LIF = Slow;
int LFC = Fast, LFI = Medium, LFF = Medium;
int RE_L[9] = { LCC, LCI, LCF, LIC, LII, LIF, LFC, LFI, LFF };
// Right wheel
int RCC = Fast, RCI = Fast, RCF = Fast;
int RIC = Medium, RII = Slow, RIF = Medium;
int RFC = Medium, RFI = Slow, RFF = Slow;
int RE_R[9] = { RCC, RCI, RCF, RIC, RII, RIF, RFC, RFI, RFF };

/// Define the rules for Obstacle Avoidance
// Left wheel
int LCCC = 0, LCCI = Slow, LCCF = Slow;
int LCIC = Medium, LCII = Slow, LCIF = Slow;
int LCFC = Medium, LCFI = Slow, LCFF = Slow;
int LICC = Medium, LICI = Slow, LICF = Slow;
int LIIC = Fast, LIII = Slow, LIIF = Slow;
int LIFC = Fast, LIFI = Medium, LIFF = Slow;
int LFCC = Fast, LFCI = Fast, LFCF = Slow;
int LFIC = Fast, LFII = Fast, LFIF = Slow;
int LFFC = Fast, LFFI = Fast, LFFF = Medium;
int OA_L[27] = { LCCC, LCCI, LCCF, LCIC, LCII, LCIF, LCFC, LCFI, LCFF,
				LICC, LICI, LICF, LIIC, LIII, LIIF, LIFC, LIFI, LIFF,
				LFCC, LFCI, LFCF, LFIC, LFII, LFIF, LFFC, LFFI, LFFF };

// Right wheel
int RCCC = Fast, RCCI = Fast, RCCF = Fast;
int RCIC = Medium, RCII = Fast, RCIF = Fast;
int RCFC = Medium, RCFI = Fast, RCFF = Fast;
int RICC = Slow, RICI = Fast, RICF = Fast;
int RIIC = Slow, RIII = Slow, RIIF = Fast;
int RIFC = Slow, RIFI = Medium, RIFF = Fast;
int RFCC = Slow, RFCI = Slow, RFCF = Fast;
int RFIC = Slow, RFII = Medium, RFIF = Fast;
int RFFC = Slow, RFFI = Slow, RFFF = Medium;
int OA_R[27] = { RCCC, RCCI, RCCF, RCIC, RCII, RCIF, RCFC, RCFI, RCFF,
				RICC, RICI, RICF, RIIC, RIII, RIIF, RIFC, RIFI, RIFF,
				RFCC, RFCI, RFCF, RFIC, RFII, RFIF, RFFC, RFFI, RFFF };