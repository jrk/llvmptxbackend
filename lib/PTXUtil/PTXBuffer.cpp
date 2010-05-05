#define DEBUG_TYPE "ptx-buffer"

#include "llvm/Instructions.h"
#include "llvm/LLVMContext.h"
#include "llvm/Module.h"
#include "llvm/Pass.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "EmbedPTX.h"

using namespace llvm;

static cl::opt<std::string> PTXTarget("ptx-target", cl::init(""), cl::Hidden,
                                      cl::desc("The global to lower to ptx"));

namespace liberty {

  class PTXBuffer {
  private:
    typedef DenseSet<const Value *> ValueSet;
    typedef DenseMap<const Value*, Value*> ValueMap;
    ValueMap valueMap;
    Module &origMod;
    std::auto_ptr<Module> bufMod;
    ValueSet users;

    const User *translate(const User *oldValue) const {
      const ValueMap::const_iterator newValue = valueMap.find(oldValue);
      assert(newValue != valueMap.end());
      return cast<const User>(newValue->second);
    }

    void insertUser(const Function *func) {
      for(const_inst_iterator inst = inst_begin(func);
          inst != inst_end(func);
          ++inst) {
        insertUser(&*inst);
      }
    }

    void insertUser(const User *user) {
      if(!users.insert(user).second) {
        return;
      }

      if(const Function *func = dyn_cast<Function>(user)) {
        insertUser(func);
      }

      typedef User::const_op_iterator OpIt;
      for(OpIt op = user->op_begin(); op != user->op_end(); ++op) {
        if(const User *child = dyn_cast<User>(op)) {
          insertUser(child);
        }
      }
    }

  public:
    PTXBuffer(Module &M) : valueMap(),
                           origMod(M),
                           bufMod(CloneModule(&M, valueMap)) {}

    bool contains(const GlobalValue *global) const {
      return users.count(translate(global));
    }

    void insert(const GlobalValue *global) {
      insertUser(translate(global));
    }

    GlobalValue *compile() const {

      typedef Module::FunctionListType FunList;
      typedef FunList::iterator FunListIt;

      FunList &funcs = bufMod->getFunctionList();
      for(FunListIt func = funcs.begin(); func != funcs.end(); ) {
        FunListIt curr = func++;
        if(!users.count(&*curr)) {
          curr->dropAllReferences();
          funcs.remove(curr);
        }
      }

      typedef Module::GlobalListType VarList;
      typedef VarList::iterator VarListIt;

      VarList &vars = bufMod->getGlobalList();
      for(VarListIt var = vars.begin(); var != vars.end(); ) {
        VarListIt curr = var++;
        if(!users.count(&*curr)) {
          curr->dropAllReferences();
          vars.remove(curr);
        }
      }

      return embedPTXAsm(origMod, *bufMod);
    }
  };

}

class PTXFunc : public ModulePass {

public:
  static char ID;

  PTXFunc() : ModulePass(&ID) {}

  virtual bool runOnModule(Module &M) {
    liberty::PTXBuffer ptxBuf(M);

    const GlobalValue *value = M.getNamedValue(PTXTarget);
    if(value)
      ptxBuf.insert(value);

    ptxBuf.compile();

    return true;
  }
};

char PTXFunc::ID = 0;

static RegisterPass<PTXFunc>
X("ptx-func", "Lower a function to PTX", false, false);
