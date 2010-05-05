/**
 * @file   PTXBackend.h
 * @date   08.08.2009
 * @author Helge Rhodin
 *
 *
 * Copyright (C) 2009, 2010 Saarland University
 *
 * This file is part of llvmptxbackend.
 *
 * llvmptxbackend is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * llvmptxbackend is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with llvmptxbackend.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef PTXBACKEND_H
#define PTXBACKEND_H

#include "PTXTargetMachine.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/TypeSymbolTable.h"
#include "llvm/Intrinsics.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/InlineAsm.h"
#include "llvm/Analysis/ConstantsScanner.h"
#include "llvm/Analysis/FindUsedTypes.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/IntrinsicLowering.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/InstVisitor.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Config/config.h"
#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>

#include "PTXPasses.h"

using namespace llvm;


#define PTX_TEX "__ptx_tex"
#define PTX_CONST "__ptx_const"
#define PTX_LOCAL "__ptx_local"
#define PTX_SHARED "__ptx_shared"

//namespace {
  class CBEMCAsmInfo : public MCAsmInfo {
  public:
    CBEMCAsmInfo() {
      GlobalPrefix = "";
      PrivateGlobalPrefix = "";
    }
  };

  /// PTXWriter - This class is the main chunk of code that converts an LLVM
  /// module to a C translation unit.
  class PTXWriter : public FunctionPass, public InstVisitor<PTXWriter>
  {
    formatted_raw_ostream &Out;
    IntrinsicLowering *IL;
    Mangler *Mang;
    DenseMap<const Value*, unsigned> AnonValueNumbers;
    LoopInfo *LI;
    const Module *TheModule;
    const MCAsmInfo* TAsm;
    MCContext *TCtx;
    const TargetData* TD;
    std::map<const Value *, const Value *>& parentPointers;
    unsigned FPCounter;
    unsigned NextAnonValueNumber;

  public:
    static char ID;
    static unsigned int POINTER_SIZE;

    explicit PTXWriter(formatted_raw_ostream &o, std::map<const Value *,
                       const Value *>& parentCompositePointer)
      : FunctionPass(&ID), Out(o), IL(0), Mang(0), LI(0),
        TheModule(0), TAsm(0), TD(0), parentPointers(parentCompositePointer),
        NextAnonValueNumber(0)
    {
      FPCounter = 0;
    }

    virtual const char *getPassName() const { return "PTX backend"; }

    void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequired<LoopInfo>();
      AU.setPreservesAll();
    }

    virtual bool doInitialization(Module &M);

    bool runOnFunction(Function &F) {
      LI = &getAnalysis<LoopInfo>();

      printFunction(F);
      return false;
    }

    virtual bool doFinalization(Module &M) {
      // Free memory...
      delete IL;
      delete TD;
      delete TAsm;
      delete TCtx;
      delete Mang;

      // Ugly hack to avoid leaking memory
      delete &parentPointers;
//       FPConstantMap.clear();
//       TypeNames.clear();
//       ByValParams.clear();
//      intrinsicPrototypesAlreadyGenerated.clear();
      return false;
    }

    std::string getTypeStr(const Type *Ty,
                        bool isSigned = false,
                        bool IgnoreName = false,
                        const AttrListPtr &PAL = AttrListPtr());
    std::string getSimpleTypeStr(const Type *Ty,
                              bool isSigned);

    void printStructReturnPointerFunctionType(formatted_raw_ostream &Out,
                                              const AttrListPtr &PAL,
                                              const PointerType *Ty);

    std::string getOperandStr(const Value *Operand);

    static unsigned int getTypeBitSize(const Type* Ty)
    { //TODO: use existing function (TD_>??)
      switch (Ty->getTypeID()) {
      case Type::VoidTyID:    assert(false && "void type?????");
      case Type::PointerTyID: return POINTER_SIZE;
      case Type::IntegerTyID: return cast<IntegerType>(Ty)->getBitWidth();
      case Type::FloatTyID:   return 32;
      case Type::DoubleTyID:  return 64;
      case Type::VectorTyID:
      {
        const VectorType* VecTy = cast<VectorType>(Ty);
        return VecTy->getNumElements()
          * getTypeBitSize(VecTy->getElementType());
      }
//        case Type::ArrayTyID:
//       {
//         const ArrayType* ArrTy = cast<ArrayType>(Ty);
//         unsigned int size = ArrTy->getNumElements()
//                        * getTypeBitSize(ArrTy->getElementType());
//       }
//       case Type::StructTyID:
//       {
//        const StructType* StrTy = cast<StructType>(Ty);
//        unsigned int size = 0;
//        for(unsigned int subtype=0; subtype < StrTy->getNumElements();
//                  subtype++)
//        {
//          const Type* elementType = StrTy->getElementType(subtype);
//          unsigned int align = getAlignmentByte(elementType);
//          size += 8 * getPadding(size*8, align);
//          size += getTypeBitSize(elementType);
//        }
//        return size;
//       }
      case Type::ArrayTyID:
      {
        const ArrayType* ArrTy = cast<ArrayType>(Ty);
        const Type* elementType = ArrTy->getElementType();
        unsigned int size_element = getTypeBitSize(elementType);
        unsigned int size = ArrTy->getNumElements() * size_element;
        unsigned int align = 8 * getAlignmentByte(elementType);
        /*
        if(size == 0)
          ArrTy->dump();
        assert(size!=0 && "no multiple of 8");
        */

        size += (ArrTy->getNumElements()-1) * getPadding(size_element, align);
        //-1 because the last element needs no "fillup"
        return size;
      }
      case Type::StructTyID:
      {
        const StructType* StrTy = cast<StructType>(Ty);
        unsigned int size = 0;
        for(unsigned int subtype=0; subtype < StrTy->getNumElements();
            subtype++)
        {
          const Type* elementType = StrTy->getElementType(subtype);
          unsigned int align = 8 * getAlignmentByte(elementType);
          size += getPadding(size, align);
          size += getTypeBitSize(elementType);
        }
        return size;
      }
      default:
        errs() << "Unknown type" <<  *Ty << "\n";
        abort();
      }
    }

    static unsigned int getTypeByteSize(const Type* Ty)
    {
      unsigned int size_bit = getTypeBitSize(Ty);
      assert((size_bit%8==0) && "no multiple of 8");
      return size_bit/8;
    }

    static unsigned int getAlignmentByte(const Type* Ty)
    {
      const unsigned int MAX_ALIGN = 8; //maximum size is 8 for doubles

      switch (Ty->getTypeID()) {
      case Type::VoidTyID:    assert(false && "void type?????");
      case Type::VectorTyID:
        return getAlignmentByte(cast<VectorType>(Ty)->getElementType());
      case Type::PointerTyID:
      case Type::IntegerTyID:
      case Type::FloatTyID:
      case Type::DoubleTyID:
        return getTypeBitSize(Ty)/8;
      case Type::ArrayTyID:
        return getAlignmentByte(cast<ArrayType>(Ty)->getElementType());
      case Type::StructTyID:
      {
        const StructType* StrTy = cast<StructType>(Ty);
        unsigned int maxa = 0;
        for(unsigned int subtype=0; subtype < StrTy->getNumElements();
            subtype++)
        {
          maxa = std::max(getAlignmentByte(StrTy->getElementType(subtype)),
                          maxa);
          if(maxa==MAX_ALIGN)
            return maxa;
        }
        return maxa;
      }
      default:
        errs() << "Unknown type" <<  *Ty << "\n";
        abort();
      }
    }

    static unsigned int getPadding(unsigned int offset, unsigned int align)
    {
      //second % needed if offset == 0
      return (align - (offset % align)) % align;
    }

    std::string getSignedConstOperand(Instruction *Operand, unsigned int op);

  private :

    std::string getTmpValueName(const Type *Ty, int index = 0)
    {
      std::stringstream name_tmp;
      name_tmp << "ptx_tmp"
               << getTypeStr(Ty)
               << '_' << index;
      std::string name = name_tmp.str();

      while(name.find(".") !=std::string::npos)
        name.replace(name.find("."),1,"_");
      return name;
    }

    bool hasSignedOperand(Instruction &I)
    {
        for(unsigned int op=0; op < I.getNumOperands(); op++)
        {
          if(isSignedOperand(I,op))
                return true;
        }
        return false;
    }

    //returns number of signed operands
    bool isSignedOperand(Instruction &I, unsigned int op_number)
    {
      switch (I.getOpcode())
      {
        default:
          return false;

        case Instruction::SDiv:
          //        case Instruction::LShr: logical shift => zero extension
        case Instruction::AShr: //arithmetic shift => sign extension
        case Instruction::SRem:
          return op_number<2;
        case Instruction::SExt:
        case Instruction::SIToFP:
          return op_number<1;
      }
    };

    bool isSignedDestinationInstruction(Instruction &I)
    {
      switch (I.getOpcode())
      {
        default:
          return false;

        case Instruction::SDiv:
        case Instruction::LShr:
        case Instruction::SRem:
          return true;
        case Instruction::SExt:
        case Instruction::FPToSI:
          return true;
      }
    };

    void moveConstantOperators(Instruction* I, unsigned int op, bool isSigned)
    {
      if(isa<Constant>(I->getOperand(op)))
      {
        const Type* Ty = I->getOperand(op)->getType();
        Out << "  mov"
            << getTypeStr(Ty, isSigned)
        // Out << ' ' << getTmpValueName(Ty,isSigned,op) << ", ";
            << ' ' << getTmpValueName(Ty,op) << ", "
            << getOperandStr(I->getOperand(op))
            << ";\n";
      }
    }

    std::string InterpretASMConstraint(InlineAsm::ConstraintInfo& c);

    void lowerIntrinsics(Function &F);

    void printModule(Module *M);
    void printModuleTypes(const TypeSymbolTable &ST);
    void printContainedStructs(const Type *Ty, std::set<const Type *> &);
    void printFunctionSignature(const Function *F, bool Prototype);

    void printFunction(Function &);
    void printEntryFunctionSignature(const Function *F, bool Prototype);
    void printFunctionArguments(const Function *F, bool Prototype,
                                bool entryFunction);
    void loadEntryFunctionParams(const Function *F);
    void printBasicBlock(BasicBlock *BB);

    void printCast(unsigned opcode, const Type *SrcTy, const Type *DstTy);
    std::string getConstant(const Constant *CPV, int dept);
    std::string getConstantCompositeAsArray(const Constant *C, int dept);
    void printConstantWithCast(const Constant *CPV, unsigned Opcode);
    bool printConstExprCast(const ConstantExpr *CE, bool Static);
    std::string getConstantArray(const ConstantArray *CPA, bool Static);
    std::string getConstantVector(const ConstantVector *CV, bool Static);

    /// isAddressExposed - Return true if the specified value's name needs to
    /// have its address taken in order to get a C value of the correct type.
    /// This happens for global variables, byval parameters, and direct allocas.
    bool isAddressExposed(const Value *V) const {
      assert(0 && "not implemented!");

    }

    // isInlinableInst - Attempt to inline instructions into their uses to build
    // trees as much as possible.  To do this, we have to consistently decide
    // what is acceptable to inline, so that variable declarations don't get
    // printed and an extra copy of the expr is not emitted.
    //
    static bool isInlinableInst(const Instruction &I) {
      assert(0 && "not implemented!");
    }

    // isDirectAlloca - Define fixed sized allocas in the entry block as direct
    // variables which are accessed with the & operator.  This causes GCC to
    // generate significantly better code than to emit alloca calls directly.
    //
    static const AllocaInst *isDirectAlloca(const Value *V) {
      const AllocaInst *AI = dyn_cast<AllocaInst>(V);
      if (!AI) return false;
      if (AI->isArrayAllocation())
        return 0;   // FIXME: we can also inline fixed size array allocas!
      if (AI->getParent() != &AI->getParent()->getParent()->getEntryBlock())
//         return 0;
      return AI;
    }

    // isInlineAsm - Check if the instruction is a call to an inline asm chunk
    static bool isInlineAsm(const Instruction& I) {
      if (isa<CallInst>(&I) && isa<InlineAsm>(I.getOperand(0)))
        return true;
      return false;
    }

    // Instruction visitation functions
    friend class InstVisitor<PTXWriter>;

    void visitReturnInst(ReturnInst &I);
    void visitBranchInst(BranchInst &I);
    void visitSwitchInst(SwitchInst &I);
    void visitInvokeInst(InvokeInst &I) {
      assert(0 && "Lowerinvoke pass didn't work!");
    }

    void visitUnwindInst(UnwindInst &I) {
      assert(0 && "Lowerinvoke pass didn't work!");
    }
    void visitUnreachableInst(UnreachableInst &I);

    void visitPHINode(PHINode &I);
    void visitBinaryOperator(Instruction &I);
    void visitICmpInst(ICmpInst &I);
    void visitFCmpInst(FCmpInst &I);

    void visitCastInst (CastInst &I);
    void visitCmpInst(CmpInst &I);
    void visitSelectInst(SelectInst &I);
    void visitCallInst (CallInst &I);
    void visitInlineAsm(CallInst &I);
    bool visitBuiltinCall(CallInst &I, bool &WroteCallee);

    void visitAllocaInst(AllocaInst &I);
    void visitLoadInst  (LoadInst   &I);
    void visitStoreInst (StoreInst  &I);
    void visitGetElementPtrInst(GetElementPtrInst &I);
    void visitVAArgInst (VAArgInst &I);

    void visitInsertElementInst(InsertElementInst &I);
    void visitExtractElementInst(ExtractElementInst &I);
    void visitShuffleVectorInst(ShuffleVectorInst &SVI);

    void visitInsertValueInst(InsertValueInst &I);
    void visitExtractValueInst(ExtractValueInst &I);

    void visitInstruction(Instruction &I) {
      errs() << "C Writer does not know about " << I;
      abort();
    }

    bool allowsConstantOperator(Instruction *v, unsigned int op)
    {
      int opcode = v->getOpcode();

      bool allowed = isa<PHINode>(v)
        || isa<ReturnInst>(v)
        || isa<AllocaInst>(v)
        || isa<LoadInst>(v)

        //shift amount can be constant!
        || ((opcode == Instruction::Shl
             || opcode == Instruction::LShr
             || opcode == Instruction::AShr)
            && op == 1)
        // store <ty> <value>, <ty>* <pointer>[, align <alignment>]
        || (op == 1 && isa<StoreInst>(v))
        // <result> = [tail] call [cconv] [ret attrs] <ty> [<fnty>*]
        //            <fnptrval>(<function args>) [fn attrs]
        || (op == 0 && isa<CallInst>(v))
        // wrapper for global variables
        || (isa<CallInst>(v)
            && v->getOperand(0)->getName().str().compare("constWrapper")==0)
        //<result> = extractelement <n x <ty>> <val>, i32 <idx>    ; yields <ty>
        || (op == 1 && isa<ExtractElementInst>(v))
        //texture access
        || (op == 1 && isa<CallInst>(v)
            && cast<CallInst>(v)->getCalledFunction()->getName().find(PTX_TEX)
               != std::string::npos); // base addr. of texture fetch

      return allowed;

      //TODO: allow constants in binary operatory why not working???
      switch (v->getOpcode())
      {
        case Instruction::Add:
        case Instruction::FAdd:
        case Instruction::Sub:
        case Instruction::FSub:
        case Instruction::Mul:
        case Instruction::FMul:
        case Instruction::UDiv:
        case Instruction::SDiv:
        case Instruction::FDiv:
        case Instruction::And:
        case Instruction::Or:
        case Instruction::Xor:
        case Instruction::Shl:
        case Instruction::LShr:
        case Instruction::AShr:
        case Instruction::URem:
        case Instruction::SRem:
        case Instruction::FRem:
          return true;
//   case Instruction::FRem: Out << " % "; break;
        default:
          return allowed;
      }
    }

    void defineRegister(std::string name, const Type *Ty, Value *v,
                        bool isSigned = false)
    {
      Out << "  .reg ";
      if(isa<CmpInst>(v) && Ty->getPrimitiveSizeInBits()==1) //predicate?!
        Out << ".pred";
      else
        Out << getTypeStr(Ty,isSigned);
      Out << ' ' << name << ";\n";
    }

    bool isEntryFunction(const Function *F)
    {
      // no users and no return type => entry func
      return (F->use_begin() == F->use_end()
         && F->getReturnType()->getTypeID() == Type::VoidTyID);

    }

    const Value* getParentPointer(const Value* ptr)
    {
      if(parentPointers.find(ptr)!=parentPointers.end())
        return parentPointers.find(ptr)->second;
      else // no parent pointer => this is a parent
        return ptr;
    }

    //determines adress space from underlying object name
    std::string getAddressSpace(const Value *v)
    {
      // getUnderlyingObject resolves normal GEP instructions and casts,
      // getParentPointer resolves "replaced"(by add/mul inst.) GEP instr.
      //TODO: do iteratively till result is stable??
      const Value* parent =
        getParentPointer(getParentPointer(v)->getUnderlyingObject());

      //allocated data is stored in local memory
      if(isa<AllocaInst>(parent))
        return ".local";
      else if(getValueName(parent).find(PTX_CONST)!=std::string::npos)
        return ".const";
      else if(getValueName(parent).find(PTX_SHARED)!=std::string::npos)
        return ".shared";
      else if(getValueName(parent).find(PTX_TEX)!=std::string::npos)
        return ".tex";

      return ".global";
    }

    bool isSpecialRegister(const Value *v)
    {
      std::string sourceName = getValueName(v);
      return sourceName.find("__ptx_sreg_") == 0;// =std::string::npos;
    }

    std::string getSpecialRegisterName(const Value *v)
    {
      std::string sourceName = getValueName(v);
      if(!sourceName.find("__ptx_sreg_") == 0)// != std::string::npos)
        assert(false && "this is not an special register!");

      std::string sregName;
      //determine type
      if(sourceName.find("_tid_") != std::string::npos)
        sregName = "%tid.";
      else if(sourceName.find("_ntid_") != std::string::npos)
        sregName = "%ntid.";
      else if(sourceName.find("_ctaid_") != std::string::npos)
        sregName = "%ctaid.";
      else if(sourceName.find("_nctaid_") != std::string::npos)
        sregName = "%nctaid.";
      else if(sourceName.find("_gridid") != std::string::npos)
        return "%gridid";
      else if(sourceName.find("_clock") != std::string::npos)
        return "%clock";
      else
        assert(false && "not implemented");

      //get x,y,z
      if(sourceName.find("id_x") != std::string::npos)
        sregName += 'x';
      else if(sourceName.find("id_y") != std::string::npos)
        sregName += 'y';
      else if(sourceName.find("id_z") != std::string::npos)
        sregName += 'z';

      return sregName;
    }

    std::string getPredicate(Value* predicate, bool negated)
    {
      std::string pred = "";
      if(predicate!=0)
      {
        if(negated)
          pred = "@!";
        else
          pred = " @";
        pred.append(getOperandStr(predicate));
      }
      return pred;
    }

    bool isGotoCodeNecessary(BasicBlock *From, BasicBlock *To);
    void printPHICopiesForSuccessor(BasicBlock *CurBlock,
                                    BasicBlock *Successor,
                                    Value* predicate = 0, bool = false);
    void printBranchToBlock(BasicBlock *CurBlock, BasicBlock *SuccBlock,
                            std::string predicate = "");
    void printGEPExpressionStep(GetElementPtrInst &Ptr,
                                const CompositeType *CompTy,
                                unsigned int operand, bool usedGepReg);
    std::string getConstantGEPExpression(const User *Ptr);

    std::string getValueName(const Value *Operand);
  };

#endif
