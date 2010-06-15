/**
 * @file   PTXTargetMachine.h
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
 *
 * This library converts LLVM code to PTX code
 *
 * @TODO: test pass which checks code for recursion etc., struct alignment,
 *        no malloc, no assembler
 * @TODO: .uni in branch and function calls
 * @TODO: struct return, local memory...
 * @TODO: struct with different sizes (char & int => alignment?)
 * @TODO: approx still needed? write function instructionNeedsApprox()
 * @TODO: shift operations only take 32 bit operands as offset
 * @TODO: use intrinsics for threadid.x etc.
 */
#include "PTXBackend.h"
#include "PTXPasses.h"

// Register the target.
static RegisterTargetMachine<PTXTargetMachine> X(ThePTXBackendTarget);

using namespace llvm;

char PTXWriter::ID = 0;
unsigned int PTXWriter::POINTER_SIZE = 32;

std::string PTXWriter::getSimpleTypeStr(const Type *Ty, bool isSigned)
{
  assert((Ty->isPrimitiveType() || Ty->isIntegerTy() || isa<VectorType>(Ty)) &&
	 "Invalid type for getSimpleType");
  switch (Ty->getTypeID()) {
  case Type::VoidTyID:
  {
    assert(false && "void type?????");
    return ".void";
  }
  case Type::IntegerTyID: {
    unsigned NumBits = cast<IntegerType>(Ty)->getBitWidth();
    if (NumBits == 1)
    {
      return ".pred";
    }
    else
    {
      if(!(NumBits == 8 || NumBits == 16 || NumBits == 32 || NumBits == 64))
	Ty->dump();
      assert((NumBits == 8 || NumBits == 16 || NumBits == 32 || NumBits == 64)
	     && "Bitsize not supported!!");

      // Use of 8-bit registers is highly restricted.
      if(NumBits == 8) {
	NumBits = 16;
      }
      std::stringstream tmpStream;
      tmpStream << '.' <<  (isSigned?"s":"u") << NumBits;
      return tmpStream.str();
    }
  }
  case Type::FloatTyID:
    return ".f32";
  case Type::DoubleTyID:
    return ".f64";
  case Type::VectorTyID:
  {
    const VectorType *VTy = cast<VectorType>(Ty);
      std::stringstream tmpStream;
      tmpStream << ".v"
		<< VTy->getNumElements()
		<< getSimpleTypeStr(VTy->getElementType(), isSigned);

      return tmpStream.str();
    //Out << ".b" << getTypeBitSize(VTy->getElementType());
  }

  default:
    errs() << "Unknown primitive type: " << *Ty << "\n";
    abort();
  }
}

std::string PTXWriter::getTypeStr(const Type *Ty,
				 bool isSigned,
				 bool IgnoreName, const AttrListPtr &PAL)
{
  if (Ty->isPrimitiveType() || Ty->isIntegerTy() || isa<VectorType>(Ty)) {
    return getSimpleTypeStr(Ty, isSigned);
  }

  switch (Ty->getTypeID()) {
  case Type::PointerTyID:
  {
    // all pointers are represented throught POINTER_SIZE=32 bit unsigned int
    // addresses
    return getTypeStr(IntegerType::get(Ty->getContext(), POINTER_SIZE), false);
  }

  case Type::ArrayTyID:
  case Type::StructTyID:
  {
    std::stringstream name_tmp;
    name_tmp <<  ".align " << getAlignmentByte(Ty) << " .b8";
    return name_tmp.str();
  }
  case Type::OpaqueTyID: {
    assert(false && " opaque not implemented");
  }
  default:
    errs() << "Unhandled case in getTypeProps! " << *Ty
	   << " | " << Ty->getTypeID() << "\n";
    //    assert(0 && "Unhandled case in getTypeProps!");
    abort();
  }
}

// printConstant - The LLVM Constant to C Constant converter.
// TODO: dept
std::string PTXWriter::getConstant(const Constant *CPV, int dept = 0)
{
  // undef value is allowed to have an arbitrary value, here 0.
  // TODO: better solution eg. any used register?
  if(isa<UndefValue>(CPV)) {
    const Type *type = CPV->getType();
    if(type->isDoubleTy() || type->isFloatTy())
      return "0.0";
    else if(type->isIntegerTy())
      return "0";
    else
      assert(false && "TODO what about undef structs,floats...??");
  }

  if(const ConstantExpr* constExp = dyn_cast<ConstantExpr>(CPV))
  {
    switch (constExp->getOpcode())
    {
      default:
	errs() << "================== WARNING: ConstantExp: "
	       << constExp->getOpcodeName()
	       << " (unsupported), replaced by getOperand(0) TODO:==========\n";
	constExp->dump();
	errs() << "=========================================================\n";

	return getConstant(dyn_cast<Constant>(constExp->getOperand(0)));
      case Instruction::GetElementPtr:
	return getConstantGEPExpression(constExp);
      case Instruction::BitCast:
	//<result> = bitcast <ty> <value> to <ty2> ; yields ty2
      case Instruction::PtrToInt:
      case Instruction::IntToPtr:
	return getConstant(dyn_cast<Constant>(constExp->getOperand(0)));
      }
  }

  unsigned int typeID = CPV->getType()->getTypeID();
  if(dept>0) //we are stepping down in a struct or array
  {
    APInt Int;
    switch (typeID)
    {
      case Type::IntegerTyID:
	Int = cast<ConstantInt>(CPV)->getValue();
	break;
      case Type::FloatTyID:
      case Type::DoubleTyID:
	Int = cast<ConstantFP>(CPV)->getValueAPF().bitcastToAPInt();
	break;
      default:

	CPV->dump();
	errs() << "typeID " << typeID << " " << CPV << "\n";
	//        return "XXXX";
	//        assert(false && typeID && "not implemented");
    }
    bool wrote = false;
    unsigned int bitwidth = Int.getBitWidth();

    std::stringstream Out;
    for(unsigned int i=0; i<bitwidth; i+=8)
    {
      if(wrote)
	Out << ", ";
      wrote = true;
      Out << Int.getLoBits(i+8).getHiBits(bitwidth-i).getZExtValue();
    }
    return Out.str();
  }

  //simple type or dept 0
  switch (typeID)
  {
    case Type::IntegerTyID:
    {
      const ConstantInt *CI = cast<ConstantInt>(CPV);
      std::stringstream Out;
      Out << CI->getZExtValue();
      return Out.str();
    }
    case Type::FloatTyID:
    {
      const ConstantFP *FPC = cast<ConstantFP>(CPV);
      char buf[30];
      sprintf (buf, "%f", FPC->getValueAPF().convertToFloat());
      std::stringstream Out;
      Out << buf;
      return Out.str();
    }
    case Type::DoubleTyID:
    {
      const ConstantFP *FPC = cast<ConstantFP>(CPV);
      char buf[30];
      std::stringstream Out;
      sprintf (buf, "%f", FPC->getValueAPF().convertToDouble());
      Out << buf;
      return Out.str();
    }
    case Type::PointerTyID:
    {
      if (isa<ConstantPointerNull>(CPV))
	return "0";
      else if (const GlobalValue *GV = dyn_cast<GlobalValue>(CPV))
	return getOperandStr(GV);
      CPV->dump();
      assert(false && "not implemented");
    }
    case Type::VectorTyID:
    case Type::ArrayTyID:
    case Type::StructTyID:
    {
      //print zero array for zero initialiser
      if(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV))
      {
	//detremine element type
	const Type *elTy;
	if(typeID == Type::VectorTyID)
	{
	  const VectorType *vecTy = cast<VectorType>(CPV->getType());
	  elTy = vecTy->getElementType();
	}
	else // structs and array are only allowed during initialisation,
	     // we represent them with 8 byte unsigned
	{
	  elTy = Type::getInt8Ty(CPV->getContext());
	}
	unsigned int size = getTypeByteSize(CPV->getType());
	unsigned int elementSize = getTypeByteSize(elTy);

	std::stringstream Out;
	Out << "{ ";
	bool printed = false;
	for (unsigned i = 0; i < size; i += elementSize)
	{
	  if(printed)
	    Out << ",";
	  if(i%32==0 && i!=0)
	    Out << "\n       ";

	  Out << getConstant(Constant::getNullValue(elTy));
	  //          Out << '0';
	  printed = true;
	}
	Out << " }";
	return Out.str();
      }
      else
      {
	std::stringstream Out;

	//print nonezero struct or array
	if(dept == 0)
	  Out << "{ ";
	bool wrote = false;
	for (unsigned i = 0; i < CPV->getNumOperands(); ++i)
	{
	  if(wrote)
	    Out << ",";
	  //       Out << ",\n";
	  wrote = true;
	  Out << getConstant(cast<Constant>(CPV->getOperand(i)), dept+1);
	}
	if(dept == 0)
	  Out << " }";
	return Out.str();
      }
    }
    case Type::OpaqueTyID:
      assert(false && "Opaque type id not implemented");
    default:
      errs() << "======================= type not implemented: "
	     << *CPV->getType()
	     << " | " << CPV->getType()->getTypeID() << "\n";
      CPV->dump();
      assert(false && "type not implemented");
      return  "IGNOREED_UNKNOWN";
  }
}

static std::string CBEMangle(const std::string &S) {
  std::string Result;

  for (unsigned i = 0, e = S.size(); i != e; ++i)
    if (isalnum(S[i]) || S[i] == '_') {
      Result += S[i];
    } else {
      Result += '_';
      Result += 'A'+(S[i]&15);
      Result += 'A'+((S[i]>>4)&15);
      Result += '_';
    }
  return Result;
}

std::string PTXWriter::getValueName(const Value *Operand) {
  std::string Name;

  // Mangle globals with the standard mangler interface for LLC compatibility.
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
    SmallString<128> Str;
    Mang->getNameWithPrefix(Str, GV, false);
    return CBEMangle(Str.str().str());
  }

  std::string VarName;

  Name = Operand->getName();

  if (Name.empty()) { // Assign unique names to local temporaries.
    unsigned &No = AnonValueNumbers[Operand];
    if (No == 0)
      No = ++NextAnonValueNumber;
    Name = "tmp__" + utostr(No);
  }


  VarName.reserve(Name.capacity());

  for (std::string::iterator I = Name.begin(), E = Name.end();
       I != E; ++I) {
    char ch = *I;

    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
	  (ch >= '0' && ch <= '9') || ch == '_')) {
      char buffer[5];
      sprintf(buffer, "_%x_", ch);
      VarName += buffer;
    } else
      VarName += ch;
  }

  Name = "llvm_ptx_" + VarName;
  return Name;
}

//TODO: dont call getOperandStr directly!
std::string PTXWriter::getOperandStr(const Value *Operand)
{
  const Constant* CPV = dyn_cast<Constant>(Operand);

  if (CPV && !isa<GlobalValue>(CPV))
  {
    return getConstant(CPV);
  }
  else if(isa<ArrayType>(Operand->getType()))
    assert(false && "array operand not implemented");
  else
    return getValueName(Operand);
}

std::string PTXWriter::getSignedConstOperand(Instruction *I, unsigned int op)
{
  Value *Operand = I->getOperand(op);
  if(isa<Constant>(Operand) && !allowsConstantOperator(I,op))
    return getTmpValueName(Operand->getType(),op);
  return getOperandStr(Operand);
}

enum SpecialGlobalClass {
  NotSpecial = 0,
  GlobalCtors, GlobalDtors,
  NotPrinted
};

// getGlobalVariableClass - If this is a global that is specially recognized
// by LLVM, return a code that indicates how we should handle it.
static SpecialGlobalClass getGlobalVariableClass(const GlobalVariable *GV) {
  // If this is a global ctors/dtors list, handle it now.
  if (GV->hasAppendingLinkage() && GV->use_empty()) {
    if (GV->getName() == "llvm.global_ctors")
      return GlobalCtors;
    else if (GV->getName() == "llvm.global_dtors")
      return GlobalDtors;
  }

  // Otherwise, it it is other metadata, don't print it.  This catches things
  // like debug information.
  if (GV->getSection() == "llvm.metadata")
    return NotPrinted;

  return NotSpecial;
}


bool PTXWriter::doInitialization(Module &M) {

  //  M.dump();

  //print ptx version and target
  Out << ".version 1.4\n";
  Out << ".target sm_13\n\n";

  // Initialize
  TheModule = &M;

  TD = new TargetData(&M);
  IL = new IntrinsicLowering(*TD);
  IL->AddPrototypes(M);

  // Ensure that all structure types have names...
  TAsm = new CBEMCAsmInfo();
  TCtx = new MCContext(*TAsm);
  Mang = new Mangler(*TCtx, *TD);

  //  Mang->markCharUnacceptable('.'); //TODO: what does this do?

  // Output the global variable declarations
  if (!M.global_empty())
  {
    Out << "\n// Global Variable Declarations\n";

    for (Module::global_iterator I = M.global_begin(), E = M.global_end();
	 I != E; ++I)
    {

      if (!I->isDeclaration())
      {
	// Ignore special globals, such as debug info.
	if (getGlobalVariableClass(I))
	  continue;

	//dont print special registers
	if(isSpecialRegister(I))
	  continue;

	// .const/.global ...?
	Out << getAddressSpace(I);

	//print Type
	const Type* Ty = I->getType()->getElementType();
	Out << getTypeStr(Ty, false);

	//value name
	Out << ' ' << getValueName(I);

	// write dimension if array or struct
	// (structs are represented with b8 arrays)
	if(isa<ArrayType>(Ty) || isa<StructType>(Ty))
	  Out << '[' << getTypeByteSize(Ty) << ']';

	//initialisation
	if(I->getInitializer()
	   && getValueName(I).find(PTX_TEX)==std::string::npos
	   && getValueName(I).find(PTX_SHARED)==std::string::npos)
	{
	  Out << " = "
	      << getOperandStr(I->getInitializer());
	}

	//print type as comment
	Out << "; // ";
	I->getType()->getElementType()->print(Out);
	Out << "\n";
      }
    }
  }

  if (!M.empty())
    Out << "\n\n// Function Bodies \n";

  Out << '\n';

  return false;
}

void PTXWriter::printFunctionArguments(const Function *F, bool Prototype,
				       bool entryFunction)
{
  const AttrListPtr &PAL = F->getAttributes();


  bool PrintedArg = false;
  if (!F->isDeclaration())
  {
    if (!F->arg_empty())
    {
      Out << " (";
      Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      unsigned Idx = 1;

      std::string ArgName = "";
      for (; I != E; ++I)
      {
	ArgName = " ";
	if (PrintedArg)
	  Out << ",\n         ";
	if (I->hasName() || !Prototype)
	  ArgName.append(getValueName(I));
	else
	{
	  assert(false && "no name for argument");
	  ArgName = " ";
	}

	if(entryFunction)
	  // append _par suffix to name, because load from .par to final
	  // variable is nessesary
	  ArgName.append("_par");

	const Type *ArgTy = I->getType();
	 if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
	   assert(isa<PointerType>(ArgTy));
	   ArgTy = cast<PointerType>(ArgTy)->getElementType();
	   errs() << "WARNING: encountered by val argument!!!!!!!!\n";
	   //                      assert(false && "whats this? ajdh");
	 }   //TODO: test

	//add space prefix
	if(entryFunction)
	  Out << ".param ";
	else
	  Out << ".reg ";

	//print type and name
	Out << getTypeStr(ArgTy,false) << ArgName;
	PrintedArg = true;
	++Idx;
      }
      Out << ')';
    }
  }
  else
  {
    assert(false && "declaration not supported");
  }
}

void PTXWriter::printEntryFunctionSignature(const Function *F, bool Prototype)
{
  Out << ".entry ";//start signature

  // Print out the name...
  Out << getValueName(F);

  printFunctionArguments(F, Prototype, true);
  Out << "\n";
}

void PTXWriter::printFunctionSignature(const Function *F, bool Prototype)
{
  Out << ".func ";//start signature

  //return type
  const Type *RetTy = F->getReturnType();
  if(RetTy->getTypeID()!=Type::VoidTyID)
  {
    Out << "(.reg"
	<< getTypeStr(RetTy)
	<< " llvm_ptx__returnreg) ";
  }
  Out << getValueName(F) << " ";

  printFunctionArguments(F, Prototype, false);
  Out << "\n";
}

void PTXWriter::loadEntryFunctionParams(const Function *F)
{
  if (!F->arg_empty())
  {
    Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();

    Out << "  // Loading function parameters from .param \n";
    for (; I != E; ++I)
    {
      //define gegister
      Out << "  .reg "
	  << getTypeStr(I->getType())
	  << ' ' << getValueName(I) << ";\n";

      //load param value
      Out << "  ld.param"
	  << getTypeStr(I->getType())
	  << ' ' << getValueName(I) << ", [" //destination
	  << getValueName(I) << "_par];\n"; //source
    }
    Out << '\n';
  }
}

void PTXWriter::printFunction(Function &F)
{
  // Don't bother with static constructors, PTX doesn't support them anyway.
  if(F.getName().startswith("_GLOBAL__"))
    return;

  //assume that every device function is inlined => only "__global__" functions
  if(isEntryFunction(&F))
    printEntryFunctionSignature(&F, false);
  else
    printFunctionSignature(&F, false);


  // ====== body =======
  Out << "{\n";

  bool PrintedVar = false;
  // print local variable information for the function
  Out << "  // used registers\n";

  std::map< std::string, int> types_constant_printed;

  for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I)
  {
    //print register definitions (and PHI-tmp register)
    if (I->getType() != Type::getVoidTy(I->getContext()))
    {
      defineRegister(getValueName(&*I), I->getType(),&*I);

      if (isa<PHINode>(*I)) {  // Print out PHI node temporaries as well...
	defineRegister(getValueName(&*I)+"__PHI_TEMP", I->getType(),&*I);
      }
      PrintedVar = true;
    }

    //print register for instructions with constant operators
    for(int op=0; op<(int)I->getNumOperands(); op++)
    {
	const Type* Ty = I->getOperand(op)->getType();
	if(!allowsConstantOperator(&*I,op) && isa<Constant>(I->getOperand(op)))
	{
	  std::string key = getTypeStr(Ty);

	  if(types_constant_printed.count(key)==0)
	    types_constant_printed[key] = -1;

	  if(types_constant_printed[key]<op)
	  {
	    for(int op_iter=types_constant_printed[key]+1; op_iter<=op;
		op_iter++)
	      defineRegister(getTmpValueName(Ty,op_iter),Ty,&*I,false);
	    types_constant_printed[key] = op;
	  }
	}
    }
  }

  if (PrintedVar)
    Out << '\n';

  // device function parameters need to be loaded from .param adress space
  if(isEntryFunction(&F))
    loadEntryFunctionParams(&F);

  //print the basic blocks
  for (Function::iterator BB = F.begin(), E = F.end(); BB != E; ++BB)
    printBasicBlock(BB);

  Out << "}\n\n";
}

void PTXWriter::printBasicBlock(BasicBlock *BB)
{
  //print lable
  Out << getOperandStr(BB)
      << ":\n";

  // Output all of the instructions in the basic block...
  for (BasicBlock::iterator II = BB->begin(), E = --BB->end(); II != E;
       ++II)
  {
    //move Constant operators to registers
    for(unsigned int op=0; op<II->getNumOperands(); ++op)
    {
      // const Type *Ty = II->getOperand(op)->getType();
      bool isSignedop = isSignedOperand(*II,op);
      if(isa<Constant>(II->getOperand(op)) && !allowsConstantOperator(&*II,op))
	moveConstantOperators(&*II, op, isSignedop);
    }

    //print instruction
    visit(*II);
  }

  // Don't emit prefix or suffix for the terminator.
  visit(*BB->getTerminator());
}

void PTXWriter::visitReturnInst(ReturnInst &I)
{
  if (I.getNumOperands())
  {
    Out << "  mov"
	<< getTypeStr(I.getOperand(0)->getType())
	<< " llvm_ptx__returnreg, "
	<< getOperandStr(I.getOperand(0))
	<< ";    //move return value\n";
  }

  const Function* F = I.getParent()->getParent();
  if(isEntryFunction(F)) //entry functions do not return values
    Out << "  exit;\n";
  else
    Out << "  ret;\n";
}

void PTXWriter::printPHICopiesForSuccessor (BasicBlock *CurBlock,
					  BasicBlock *Successor,
					  Value* predicate,
					  bool negated)
{
  for (BasicBlock::iterator I = Successor->begin(); isa<PHINode>(I); ++I)
  {
    PHINode *PN = cast<PHINode>(I);
    // Now we have to do the printing.
    Value *IV = PN->getIncomingValueForBlock(CurBlock);
    if (!isa<UndefValue>(IV))
    {
      //set predicate
      Out << getPredicate(predicate,negated);

      //do move instruction
      Out << "  mov"
	  << getTypeStr(IV->getType())
	  << ' ' << getValueName(I) << "__PHI_TEMP, "
	  << getOperandStr(IV)
	  << ";    // save a copy of this register for PHI nodes\n";
    }
  }
}

void PTXWriter::printBranchToBlock(BasicBlock *CurBB, BasicBlock *Succ,
				   std::string predicate)
{
  //not direct successor => goto TODO: gives strange errors, bug in ptx?!?!
//  if (next(Function::iterator(CurBB)) != Function::iterator(Succ))
  {
    Out << predicate << "  bra "
	<< getOperandStr(Succ)
	<< ";\n";
    return;
  }
}

// Branch instruction printing - Avoid printing out a branch to a basic block
// that immediately succeeds the current one.
//
void PTXWriter::visitBranchInst(BranchInst &I) {
  if (I.isConditional())   //implemented with predicates
  {
    //first successor
    printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0),
				I.getCondition(), false);
    printBranchToBlock(I.getParent(), I.getSuccessor(0),
		       getPredicate(I.getCondition(),false));

    //second successor
    printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(1),
				I.getCondition(), true);
    printBranchToBlock(I.getParent(), I.getSuccessor(1),
		       getPredicate(I.getCondition(),true));
    return;
  }
  else
  {
    printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0));
    printBranchToBlock(I.getParent(), I.getSuccessor(0));
    return;
  }
}

// PHI nodes get copied into temporary values at the end of predecessor basic
// blocks.  We now need to copy these temporary values into the REAL value for
// the PHI.
void PTXWriter::visitPHINode(PHINode &I) {
  Out << "  mov"
      << getTypeStr(I.getType())
      << ' ' << getValueName(&I) << ", " << getValueName(&I) << "__PHI_TEMP;\n";
}

void PTXWriter::visitBinaryOperator(Instruction &I) {
  // binary instructions, shift instructions, setCond instructions.
  assert(!isa<PointerType>(I.getType()) && "no pointer type!!!!");

  //check for signed operations => need conversion
  bool isSigned = hasSignedOperand(I);

  //print instruction
  Out << "  ";
  switch (I.getOpcode())
  {
  case Instruction::Add:
  case Instruction::FAdd:  Out << "add"; break;
  case Instruction::Sub:
  case Instruction::FSub:  Out << "sub"; break;
  case Instruction::Mul:
  case Instruction::FMul:  Out << "mul"; break; //TODO: hi? wide??
  case Instruction::UDiv:
  case Instruction::SDiv:
  case Instruction::FDiv: Out << "div"; break;
  case Instruction::And:  Out << "and"; break;
  case Instruction::Or:   Out << "or"; break;
  case Instruction::Xor:  Out << "xor"; break;
  case Instruction::Shl:  Out << "shl"; break;
  case Instruction::LShr: //with zero fill
  case Instruction::AShr: //with sign extension
    Out << "shr";
    break;
  case Instruction::URem:
  case Instruction::SRem:
  case Instruction::FRem: Out << "rem"; break;
//   case Instruction::FRem: Out << " % "; break;
  default: errs() << "Invalid operator type!" << I; abort();
  }

  const Type *Ty = I.getOperand(0)->getType();
  //.lo is required, because llvm allows only multiplications of same type
  if(I.getOpcode()==Instruction::Mul)
  {
    if(isa<IntegerType>(Ty))
      Out << ".lo";
  }

  //div of floats need approx
  else if(I.getOpcode()==Instruction::FDiv) {
    if(I.getType()->isFloatTy()) {
      Out << ".approx";
    } else {
      Out << ".rn";
    }
  }

  //print type
  switch (I.getOpcode())
  {
    case Instruction::Shl:
//      assert(getTypeBitSize(I.getType())==32
//      && "shift left destination register must be of size 32 bit");
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
      //predicate is still predicate!
      if(getTypeBitSize(Ty)==1)
	Out << ".pred";
      else
	Out << ".b" << getTypeBitSize(Ty);
      break;
    default:
      Out << getTypeStr(Ty, isSigned);
  }

  //print destination and operands
  Out << ' ' << getValueName(&I)
      << ", " << getSignedConstOperand(&I, 0)
      << ", " << getSignedConstOperand(&I, 1) << ";\n"; //destination
}

void PTXWriter::visitCmpInst(CmpInst &I)
{
  Out << "  setp";

  switch (I.getPredicate()) {
  case CmpInst::FCMP_ONE:                      // != (ordered float)
  case CmpInst::ICMP_NE:  Out << ".ne"; break; // != (int)
  case CmpInst::FCMP_UNE: Out << ".neu"; break;// != (unordered float)

  case CmpInst::FCMP_OEQ:                      // == (ordered float)
  case CmpInst::ICMP_EQ:  Out << ".eq"; break; // == (int)
  case CmpInst::FCMP_UEQ: Out << ".equ"; break;// == (unordered float)

  case CmpInst::FCMP_OLE:                      // <= (ordered float)
  case CmpInst::ICMP_SLE: Out << ".le"; break; // <= (s-int)
  case CmpInst::ICMP_ULE: Out << ".ls"; break; // <= (u-int)
  case CmpInst::FCMP_ULE: Out << ".leu"; break;// <= (unordered float)

  case CmpInst::FCMP_OGE:                      // >= (ordered float)
  case CmpInst::ICMP_SGE: Out << ".ge"; break; // >= (s-int)
  case CmpInst::ICMP_UGE: Out << ".hs"; break; // >= (u-int)
  case CmpInst::FCMP_UGE: Out << ".geu"; break;// >= (unordered float)

  case CmpInst::FCMP_OLT:                      // < (ordered float)
  case CmpInst::ICMP_SLT: Out << ".lt"; break; // < (s-int)
  case CmpInst::ICMP_ULT: Out << ".lo"; break; // < (u-int)
  case CmpInst::FCMP_ULT: Out << ".ltu"; break;// < (unordered float)

  case CmpInst::FCMP_OGT:                      // > (ordered float)
  case CmpInst::ICMP_SGT: Out << ".gt"; break; // > (s-int)
  case CmpInst::ICMP_UGT: Out << ".hi"; break; // > (u-int)
  case CmpInst::FCMP_UGT: Out << ".gtu"; break;// > (unordered float)

  case CmpInst::FCMP_ORD: Out << ".num"; break;// !=NaN (ordered float)
  case CmpInst::FCMP_UNO: Out << ".nan"; break;// ==NaN (unordered float)

  default: errs() << "Invalid icmp predicate!" << I; abort();
  }

  //comparison type
  const Type *Ty = I.getOperand(0)->getType();

  //wirte type and operands
  Out << getTypeStr(Ty)
      << ' ' << getValueName(&I) << ", " //predicate destination
      << getSignedConstOperand(&I, 0)
      << ", "
      << getSignedConstOperand(&I, 1)
      << ";\n";
}

void PTXWriter::visitICmpInst(ICmpInst &I) {
  visitCmpInst(I);}
void PTXWriter::visitFCmpInst(FCmpInst &I) {
  visitCmpInst(I);}

void PTXWriter::visitCastInst(CastInst &I)
{
  //get types and names of operands
  const Type *DstTy = I.getType();
  const Type *SrcTy = I.getOperand(0)->getType();

  //check for signed instructions => special conversion needed
  bool isSignedDst = isSignedDestinationInstruction(I);
  bool isSignedSrc = hasSignedOperand(I);

  //bitcast instr.
  if(isa<BitCastInst>(I))
  {
    Out << "  mov"
	<< ".b" << getTypeBitSize(DstTy)
      //        << getTypeStr(DstTy, isSignedDst)
	<< " "
	<< getValueName(&I) << ", "
	<< getSignedConstOperand(&I,0)
	<< ";//bitcast " << getTypeStr(DstTy, isSignedDst)
			 << getTypeStr(SrcTy, isSignedDst)
	<< "\n";
    return;
  }

  if(const IntegerType* DITy = dyn_cast<IntegerType>(DstTy))
  if(DITy->getBitWidth()==1)
  {
    errs() << "ERROR: Need to implement dest predicate conversion TODO!!!?\n";
    assert(false && "not implemented");
    return;
  }
  if(const IntegerType* SITy = dyn_cast<IntegerType>(SrcTy))
  if(SITy->getBitWidth()==1)
  {
    Out << "  selp"
	<< getTypeStr(DstTy, isSignedSrc)
	<< " " << getValueName(&I)
	<< ", 1, 0, "
	<< getSignedConstOperand(&I,0)
	<< "; // select for cast expression \n";
//    errs() << "ERROR: Need to implement source predicate conversion!!!?\n";
    return;
  }

  //prints the instruction itself ".reg...."
  Out << "  cvt";

  //set rounding mode
  if(DstTy->isIntegerTy() && SrcTy->isFloatingPointTy())
    Out << ".rzi";
  else if((DstTy->isFloatingPointTy() && SrcTy->isIntegerTy())
	  || isa<FPTruncInst>(I))
    Out << ".rn"; // TODO: What to do??? Not dokumented in LLVM

  Out << getTypeStr(DstTy, isSignedDst)
      << getTypeStr(SrcTy, isSignedSrc);

  //print operants
  Out << ' '  << getValueName(&I)
      << ", " << getSignedConstOperand(&I,0) << ";\n";
}

void PTXWriter::visitSelectInst(SelectInst &I)
{
  //no selp for predicate type in selp
  if(I.getType()->getPrimitiveSizeInBits()==1)
  {
    //predicate is true => first operand
    Out << " @" << getSignedConstOperand(&I, 0)
	<< " mov"
	<< getTypeStr(I.getType())
	<< ' ' << getValueName(&I)
	<< " ," << getSignedConstOperand(&I, 1) << ";\n";

    //else second
    Out << "@!" << getSignedConstOperand(&I, 0)
	<< " mov"
	<< getTypeStr(I.getType())
	<< ' ' << getValueName(&I)
	<< ", " << getSignedConstOperand(&I, 2) << ";\n";

    Out << "//for old instruction: ";
  }

  Out << "  selp"
      << getTypeStr(I.getType())
      << ' ' << getValueName(&I) << ", " //destination
      << getSignedConstOperand(&I, 1) //source 1
      << ", "
      << getSignedConstOperand(&I, 2) //source 2
      << ", "
      << getSignedConstOperand(&I, 0) //pedicate
      << ";\n";
}

void PTXWriter::visitCallInst(CallInst &I) {
  if (isa<InlineAsm>(I.getOperand(0)))
    assert(false && "not implemented");//    return visitInlineAsm(I);

  bool WroteCallee = false;

  // Handle intrinsic function calls first...
  if (I.getCalledFunction())
    if (visitBuiltinCall(I, WroteCallee))
      return;

  Out << "  call ";

  //return vale
  if(I.getCalledFunction()->getReturnType()->getTypeID()!=Type::VoidTyID)
    Out << '(' << getValueName(&I) << "), ";

  //called function name
  Out << getValueName(I.getCalledFunction());

  //arguments //TODO: does the new code work??
  bool PrintedArg = false;
  for (unsigned int op = 1; op < I.getNumOperands(); ++op) {
    if (PrintedArg)
      Out << ", ";
    else
      Out << ", (";
    Out << getSignedConstOperand(&I,op);
    PrintedArg = true;
  }
  if(PrintedArg)
    Out << ')';
  Out << ";\n";


  /*
  CallSite::arg_iterator AI = I.op_begin()+1, AE = I.op_end();
  unsigned int op = 1;
  bool PrintedArg = false;
  for (; AI != AE; ++AI) {
    if (PrintedArg)
      Out << ", ";
    else
      Out << ", (";
    Out << getSignedConstOperand(*AI, op);
    ++op;
    PrintedArg = true;
  }
  if(PrintedArg)
    Out << ')';
 Out << ";\n";
  */
}

/// visitBuiltinCall - Handle the call to the specified builtin.  Returns true
/// if the entire call is handled, return false it it wasn't handled, and
/// optionally set 'WroteCallee' if the callee has already been printed out.
const unsigned int myintr_floor = Intrinsic::num_intrinsics+1;
const unsigned int myintr_sqrt = Intrinsic::num_intrinsics+2;

//   <result> = [tail] call [cconv] [ret attrs] <ty> [<fnty>*]
//              <fnptrval>(<function args>) [fn attrs]
bool PTXWriter::visitBuiltinCall(CallInst &I,
			       bool &WroteCallee)
{
  Function* F = I.getCalledFunction();
  std::string name = F->getName();

  //texture fetch?
  if(name.find(PTX_TEX)!=std::string::npos)
  {
    //determine dimension
    int dimension = 0;
    if(name.find("__ptx_tex1D")!=std::string::npos)
      dimension = 1;
    else if(name.find("__ptx_tex2D")!=std::string::npos)
      dimension = 2;
    else if(name.find("__ptx_tex3D")!=std::string::npos)
      dimension = 3;
    else
       assert(false && "strange texture fetch!!!");

    //determine type of ptr
    const Type *Ty =
      cast<PointerType>(I.getOperand(1)->getType())->getElementType();

    bool isSigned = true; //integers are always signed, see ptx-isa

    //print texture lookup
    Out << "  {\n";

    //print dummy register definitions
    for(int i=0; i<4-dimension; i++)
    {
      defineRegister("__ptx_tex_dummy"+utostr(i), Ty, &I);
      Out << "    mov.f32 __ptx_tex_dummy"+utostr(i)
	  << ", 0f00000000;\n";
    }

    //texture lookup
    Out << "    tex."
	<< dimension << "d" //1d,2d or 3d
	<< ".v4" //always v4, see ptx isa
	<< getTypeStr(Ty, isSigned) //dest and src type is the same (u32 or f32)
	<< getTypeStr(Ty, isSigned);

    //print operands
    Out << ' ' << getValueName(&I) << ", ["
	<< getOperandStr(I.getOperand(1))
	<< ", {";
    for(int i=0; i<4; i++) //index per dimension(vector)
    {
      if(i<dimension)
	Out << getSignedConstOperand(&I, 2+i);
      else //rest needs to be filled by dummies
	Out << "__ptx_tex_dummy"+utostr(i-dimension);
      if(i<3) //fuer U.
	Out << ", ";
    }
    Out << "}];}\n";
    return true;
  }
  //Synchronize
  else if(name.find("__syncthreads")!=std::string::npos)
  {
    Out << "  bar.sync 0;\n"; //TODO: bar nr
    return true;
  }
  //convert half float to double (half floats are loaded as short)
  else if(name.find("__half2float")!=std::string::npos)
  {
    // converting f16 to f32 needs some ugly conversions. ld.space.b16
    // & cvt.f32.b16 works, however this can't be represented in LLVM easily.
    // The following conversion was emitted by nvcc, I simply adopted it.
    Out << "  { .reg.u32 __ptx_tmp_halffloatA;\n"
	<< "    .reg.u32 __ptx_tmp_halffloatB;\n"
	<< "    cvt.u32.u16 __ptx_tmp_halffloatA, "
	<< getSignedConstOperand(&I,1) << ";\n"
	<< "    cvt.u16.u32 __ptx_tmp_halffloatB, __ptx_tmp_halffloatA;\n"

	<< "    .reg.b32 __ptx_tmp_halffloatC;\n"
	<< "    mov.b32 __ptx_tmp_halffloatC, "
	<< "    __ptx_tmp_halffloatB;\n";
      //        << getSignedConstOperand(&I,1) << ";\n";

    //write conversion instruction
    const Type *DstTy = I.getType();
    Out << "    cvt" //convert instruction
	<< getTypeStr(DstTy) //dest type
	<< ".f16" //source type
	<< ' '  << getValueName(&I)
	<< ", __ptx_tmp_halffloatC;}\n";
      //        << ", " << getSignedConstOperand(&I,1) << ";\n";

    return true;
  }

  //special cases or no Intrinsic
  unsigned int ID;
  //  unsigned int num_intrinsics_current = num_intrinsics;
  //unsigned int myintr_floor = num_intrinsics_current++;
  if (!(ID = (Intrinsic::ID)F->getIntrinsicID()))
  {
    if(name == "floorf")
      ID = myintr_floor;
    else if(name == "sqrtf" || name == "sqrt")
      ID = Intrinsic::sqrt;
    else if(name == "exp2f")
      ID = Intrinsic::exp2;
    else if(name == "sinf")
      ID = Intrinsic::sin;
    else if(name == "cosf")
      ID = Intrinsic::cos;
    else if(name == "log2f")
      ID = Intrinsic::log2;
  }

  switch (ID) {
  // floating point unary intrinsics
  case Intrinsic::exp2:
  case Intrinsic::sqrt:
  case Intrinsic::sin:
  case Intrinsic::cos:
  case myintr_floor:
  case Intrinsic::log2:
  //case Intrinsic::rsqrt:
  {
    if(I.getType()->getTypeID() != Type::FloatTyID
      && I.getType()->getTypeID() != Type::DoubleTyID)
      assert(false && "only floats");
  }
  //general instructions
  //case myintr_floor:
  {
    Out << "  ";
    // print instruction
    switch (ID) {
    case Intrinsic::log2: Out << "lg2.approx"; break;
    //case Intrinsic::rsqrt:  Out << "rsqrt"; break;
    case Intrinsic::exp2: Out << "ex2.approx"; break;
    case Intrinsic::sqrt:
      if(I.getType()->isFloatTy()) {
        Out << "sqrt.approx";
      } else {
        Out << "sqrt.rn";
      }
      break;
    case Intrinsic::sin: Out << "sin.approx"; break;
    case Intrinsic::cos: Out << "cos.approx"; break;
    case myintr_floor:
      //TODO: if previous instruction is mul,add... move rounding to that inst.?
      Out << "cvt.rmi" << getTypeStr(I.getType());
      break;
    default: assert(false && "unknown intrinsic");
    }

    // operands
    Out << getTypeStr(I.getType())
	<< ' ' << getValueName(&I) << ", "
	<< getSignedConstOperand(&I, 1) //source
	<< ";\n";
    return true;
  }

  case Intrinsic::powi:
  {
    assert(false && "powi not implemented jet TODO");
  }

  default: //no inline function, return
    return false;
  }
}

void PTXWriter::visitAllocaInst(AllocaInst &I)
{
  const Type *Ty = cast<PointerType>(I.getType())->getElementType();
  unsigned int size = getTypeByteSize(Ty);

  // prin array def.
  Out << "  " << getAddressSpace(&I)
      << getTypeStr(Ty)
      << ' ' << getValueName(&I) << "_def"
      << '[' << size << "];    // get local space for struct or array\n";

  // move adress to reg. //TODO use existing visitMovInstruction???!!
  Out << "  mov.u32 " << getValueName(&I) << ", "
      << getValueName(&I) << "_def; // move adress to register\n";
}

std::string PTXWriter::getConstantGEPExpression(const User *GEP)
{
  //calculate constant offset
  unsigned int offset = 0;
  const CompositeType* CompTy =
    cast<CompositeType>(GEP->getOperand(0)->getType());

  for(unsigned op_i=1; op_i<GEP->getNumOperands(); op_i++)
  {
    //get size of alle element types whiche are before the selected field
    ConstantInt* ConstOpValue = dyn_cast<ConstantInt>(GEP->getOperand(op_i));

    //none constant index?
    if(!ConstOpValue)
      assert(false && "should be constant1!!");

    unsigned int TypeIndex = ConstOpValue->getZExtValue();
    unsigned int size = 0;
    for(unsigned int ty_i=0; ty_i < TypeIndex; ty_i++)
      size += getTypeByteSize(CompTy->getTypeAtIndex(ty_i));

    offset += size;

    //sep down in the type "tree" ?
    if(op_i+1<GEP->getNumOperands())
      CompTy = dyn_cast<CompositeType>(CompTy->getTypeAtIndex(TypeIndex));
  }

  //return pointer with offset
  char buf[30];
  sprintf (buf, "%d", offset);
  return getValueName(GEP->getOperand(0)).append(" + ").append(buf);
}

//<result> = load <ty>* <pointer>[, align <alignment>][, !nontemporal !]
void PTXWriter::visitLoadInst(LoadInst &I)
{
  //move forspecial registers
  if(isSpecialRegister(I.getOperand(0)))
  {
    Out << "  mov"
	<< getTypeStr(I.getType())
	<< ' ' << getValueName(&I) << ", "
	<< getSpecialRegisterName(I.getOperand(0)) << ";\n";
  }
  else
  {
    //print instruction and type
    Out << "  ld"
	<< getAddressSpace(I.getOperand(0));

    // 8-bit loads use 16-bit registers so they are handled seperately.
    if(I.getType()->isIntegerTy(8))
      Out << ".u8";
    else
      Out << getTypeStr(I.getType());

    //print operands
    Out << ' ' << getValueName(&I) << ", [" //destination
	<< getOperandStr(I.getOperand(0)) //source
	<< "];\n";
  }
}

void PTXWriter::visitStoreInst(StoreInst &I)
{
  Out << "  st"
      << getAddressSpace(I.getOperand(1));

  // 8-bit stores use 16-bit registers so they are handled seperately.
  if(I.getType()->isIntegerTy(8))
    Out << ".u8";
  else
    Out << getTypeStr(I.getOperand(0)->getType());

  Out << " ["
      << getOperandStr(I.getOperand(1)) //destination
      << "], "
      << getSignedConstOperand(&I, 0)
      << ";\n";
}

//extract element from vector
void PTXWriter::visitExtractElementInst(ExtractElementInst &I)
{
  const Type *EltTy =
    cast<VectorType>(I.getOperand(0)->getType())->getElementType();

  Out << "  mov"
      << getTypeStr(EltTy)
      << " " << getValueName(&I) << ", " << getValueName(I.getOperand(0));

  std::string elementNum = getOperandStr(I.getOperand(1));
  if(elementNum == "0")
    Out << ".x";
  else if(elementNum == "1")
    Out << ".y";
  else if(elementNum == "2")
    Out << ".z";
  else
    Out << ".w";
  Out << ";\n";
}

void PTXWriter::visitGetElementPtrInst(GetElementPtrInst &I)
{
  assert(false
   && "WARNING: Ignoring GEP instruction, should be deleted beforehand!====\n");
}

///////////////////////////////////////////////////////////////////////
// unused / unimplemented functions... ////////////////////////////////
///////////////////////////////////////////////////////////////////////

void PTXWriter::lowerIntrinsics(Function &F) {
assert(false && "not implemented");}

void PTXWriter::visitVAArgInst(VAArgInst &I) {
assert(false && "not implemented");}

void PTXWriter::visitInsertElementInst(InsertElementInst &I) {
assert(false && "not implemented");}

void PTXWriter::visitShuffleVectorInst(ShuffleVectorInst &SVI) {
assert(false && "not implemented");}

void PTXWriter::visitInsertValueInst(InsertValueInst &IVI) {
assert(false && "not implemented");}

void PTXWriter::visitExtractValueInst(ExtractValueInst &EVI) {
assert(false && "not implemented");}

void PTXWriter::visitInlineAsm(CallInst &CI) {
assert(false && "not implemented");}

void PTXWriter::visitSwitchInst(SwitchInst &SI) {
assert(false && "not implemented");}

void PTXWriter::visitUnreachableInst(UnreachableInst &I) {
assert(false && "not implemented");}

void PTXWriter::printStructReturnPointerFunctionType(formatted_raw_ostream &Out,const AttrListPtr &PAL,const PointerType *TheTy) {
  assert(false && "not implemented");}

//===----------------------------------------------------------------------===//
//                       External Interface declaration
//===----------------------------------------------------------------------===//

bool PTXTargetMachine::addPassesToEmitFile(PassManagerBase &PM,
					   formatted_raw_ostream &o,
					   CodeGenFileType fileType,
					   CodeGenOpt::Level OptLevel,
					   bool DisableVerify) {
  if (fileType != TargetMachine::CGFT_AssemblyFile) return true;

  std::map<const Value *, const Value *>* parentCompositePointer = new std::map<const Value *, const Value *>;




    // Propagate constants at call sites into the functions they call.  This
    // opens opportunities for globalopt (and inlining) by substituting function
    // pointers passed as arguments to direct uses of functions.
  //    PM.add(createIPSCCPPass());

  PM.add(createFunctionInliningPass(999999));
  PM.add(createGlobalOptimizerPass());

    // Linking modules together can lead to duplicated global constants, only
    // keep one copy of each constant...
    PM.add(createConstantMergePass());

    // Remove unused arguments from functions...
    PM.add(createDeadArgEliminationPass());

    // Reduce the code after globalopt and ipsccp.  Both can open up significant
    // simplification opportunities, and both can propagate functions through
    // function pointers.  When this happens, we often have to resolve varargs
    // calls, etc, so let instcombine do this.
    PM.add(createInstructionCombiningPass());

    //PM.add(createFunctionInliningPass());
       // Inline small functions //->assertion "invalid LLVM intrinsic name"

    //PM.add(createPruneEHPass());
       // Remove dead EH info   //->assertion "invalid LLVM intrinsic name"

    PM.add(createGlobalOptimizerPass());    // Optimize globals again.
    PM.add(createGlobalDCEPass());          // Remove dead functions

    // If we didn't decide to inline a function, check to see if we can
    // transform it to pass arguments by value instead of by reference.
    PM.add(createArgumentPromotionPass());

    // The IPO passes may leave cruft around.  Clean up after them.
    PM.add(createInstructionCombiningPass());
    PM.add(createJumpThreadingPass());        // Thread jumps.
    PM.add(createScalarReplAggregatesPass()); // Break up allocas

    // Run a few AA driven optimizations here and now, to cleanup the code.
    //   PM.add(createGlobalsModRefPass());      // IP alias analysis
    PM.add(createLICMPass());               // Hoist loop invariants
    PM.add(createGVNPass());                  // Remove redundancies
    PM.add(createMemCpyOptPass());          // Remove dead memcpy's
    PM.add(createDeadStoreEliminationPass()); // Nuke dead stores

    // Cleanup and simplify the code after the scalar optimizations.
    PM.add(createInstructionCombiningPass());

    //PM.add(createJumpThreadingPass());        // Thread jumps.
    PM.add(createPromoteMemoryToRegisterPass()); // Cleanup jumpthread.

    // Delete basic blocks, which optimization passes may have killed...
    PM.add(createCFGSimplificationPass());

    // Now that we have optimized the program, discard unreachable functions...
    PM.add(createGlobalDCEPass());

  /*
   PM.add(createInstructionCombiningPass());
    PM.add(createJumpThreadingPass());        // Thread jumps.
    PM.add(createScalarReplAggregatesPass()); // Break up allocas
    PM.add(createLICMPass());                 // Hoist loop invariants   //Pass
    PM.add(createGVNPass());                  // Remove redundancies
    PM.add(createMemCpyOptPass());            // Remove dead memcpy's
    PM.add(createDeadStoreEliminationPass()); // Nuke dead stores
    PM.add(createInstructionCombiningPass());
    PM.add(createJumpThreadingPass());        // Thread jumps.
    PM.add(createPromoteMemoryToRegisterPass()); // Cleanup jumpthread.
    PM.add(createCFGSimplificationPass());

   //custom
    PM.add(createSCCPPass()); //fpm.add(createConstantPropagationPass());
    PM.add(createTailCallEliminationPass());
    PM.add(createCondPropagationPass());
    PM.add(createAggressiveDCEPass());

    //clean up again
    PM.add(createInstructionCombiningPass());
    PM.add(createReassociatePass());
    PM.add(createGVNPass());
    PM.add(createCFGSimplificationPass());
    PM.add(createAggressiveDCEPass());
      //fpm.add(createDeadCodeEliminationPass());
*/

  //do everything possible on registers
  PM.add(createPromoteMemoryToRegisterPass());
  PM.add(new PTXBackendInsertSpecialInstructions(*parentCompositePointer));

  // algebraic simplification (needed after GEP replacement)
  // but also destrois GEP replacement :/
  //  PM.add(createInstructionCombiningPass());

  //commutative simplification (simplify GEP calculations)
  PM.add(createReassociatePass());
  // simplify constants generated by GEP replacement
  // REMOVED: makes global + constant as a constant expression => error
  PM.add(createConstantPropagationPass());
  PM.add(createDeadInstEliminationPass()); // remove simplified instructions
  PM.add(createStripDeadPrototypesPass()); // remove unused functions like exp
  PM.add(createLowerSwitchPass());
  PM.add(createPromoteMemoryToRegisterPass());

//   PM.add(createGCLoweringPass());
//   PM.add(createLowerAllocationsPass(true));
//   PM.add(createLowerInvokePass());
//   PM.add(createCFGSimplificationPass());   // clean up after lower invoke.
  PM.add(new PTXPolishBeforeCodegenPass());
  PM.add(new PTXWriter(o,*parentCompositePointer));
  //  PM.add(createGCInfoDeleter());
  return false;
}
