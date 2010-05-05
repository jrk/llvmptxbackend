#ifndef EMBED_PTX_H
#define EMBED_PTX_H

#include "llvm/Module.h"

namespace liberty {
  llvm::GlobalVariable *embedPTXAsm(llvm::Module &cpuMod,
                                    const llvm::Module &gpuMod);
}

#endif /* EMBED_PTX_H */
