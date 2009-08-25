/***************************************************************************
                        Edebug.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : functions for debugging and generic printing
 ***************************************************************************/
#ifndef EDEBUG_H
#define EDEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

void Edebug(const char *, ...);
void Edbginfo(int, const char *);
#define EDBG Edbginfo(__LINE__, __FILE__), Edebug

void EDBG_ENABLE();
void EDBG_DISABLE();
void EPRINT(const char *format, ...);
void EERR(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
