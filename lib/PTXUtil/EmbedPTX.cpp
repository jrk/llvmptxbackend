#define DEBUG_TYPE "embed-ptx"

#include "llvm/Instructions.h"
#include "llvm/Module.h"
#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "EmbedPTX.h"

using namespace llvm;

static cl::opt<bool> PTXDebug("print-ptx", cl::init(false), cl::Hidden,
			      cl::desc("Print ptx"));

static TargetMachine *getPTXTarget(const Module *mod) {

  const std::string MArch = "ptx";

  // Search for the ptx backend
  const Target *TheTarget = 0;
  for (TargetRegistry::iterator it = TargetRegistry::begin(),
	 ie = TargetRegistry::end(); it != ie; ++it) {
    if (MArch == it->getName()) {
      TheTarget = &*it;
      break;
    }
  }

  assert(TheTarget && "PTXBackend not loaded");

  const Triple TheTriple(mod->getTargetTriple());
  const std::string FeaturesStr = "";

  // Create a TargetMachine
  return TheTarget->createTargetMachine(TheTriple.getTriple(), FeaturesStr);
}

namespace liberty {

  GlobalVariable *embedPTXAsm(Module &cpuMod, const Module &gpuMod) {

    // Clone the GPU module, so we don't alter it permanently while lowering it.
    std::auto_ptr<Module> mod(CloneModule(&gpuMod));

    // Create a string to hold the assembly code
    std::string AssemblyCode;
    raw_string_ostream StringOut(AssemblyCode);
    formatted_raw_ostream Out(StringOut);

    // Get a PTX TargetMachine
    const std::auto_ptr<TargetMachine> Target(getPTXTarget(mod.get()));
    assert(Target.get() && "Could not allocate target machine!");

    // Various parameters for PTX to consider when adding passes
    const TargetMachine::CodeGenFileType FileType = TargetMachine::CGFT_AssemblyFile;
    const CodeGenOpt::Level OLvl = CodeGenOpt::Aggressive;
    const bool DisableVerify = false;

    // Use the TargetMachine to add lowering passes
    PassManager PM;
    Target->addPassesToEmitWholeFile(PM, Out, FileType, OLvl, DisableVerify);

    // Lower the module to PTX
    PM.run(*mod);

    // Flush the string stream, otherwise we won't see anything
    Out.flush();

    if(PTXDebug)
      errs() << AssemblyCode;

    // Embed the assembly code as constant strings
    Constant* AssemblyCodeArray =
      ConstantArray::get(cpuMod.getContext(), AssemblyCode);
    return new GlobalVariable(cpuMod,
			      AssemblyCodeArray->getType(),
			      true,
			      GlobalValue::PrivateLinkage,
			      AssemblyCodeArray,
			      "");
  }
}

class EmbedAsm : public ModulePass {

public:
  static char ID;

  EmbedAsm() : ModulePass(&ID) {}

  virtual bool runOnModule(Module &M) {
    liberty::embedPTXAsm(M, M);
    return true;
  }
};

char EmbedAsm::ID = 0;

static RegisterPass<EmbedAsm>
X("embed-asm", "Embed assembly code as a string literal", false, false);
