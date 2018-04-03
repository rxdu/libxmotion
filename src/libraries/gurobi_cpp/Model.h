// Copyright (C) 2016, Gurobi Optimization, Inc.
// All Rights Reserved
#ifndef _MODEL_CPP_H_
#define _MODEL_CPP_H_

class GRBEnv;
class GRBCallback;

class GRBModel
{
  private:

    GRBmodel    *Cmodel;
    GRBenv      *Cenv;
    GRBCallback *cb;

    int rows;
    int cols;
    int numsos;
    int numqconstrs;
    int numgenconstrs;
    int newranges;

    int updatemode;

    vector<GRBVar> vars;
    vector<GRBConstr> constrs;
    vector<GRBSOS> sos;
    vector<GRBQConstr> qconstrs;
    vector<GRBGenConstr> genconstrs;

// only for gurobi_c++.h const GRBModel& operator=(const GRBModel &xmodel);

    GRBModel();
    int  getupdmode();
    void populate();
    int* setvarsind(const GRBVar* xvars, int size);
    int* setconstrsind(const GRBConstr* xconstrs, int size);
    int* setqconstrsind(const GRBQConstr* xqconstrs, int size);
    int* setgenconstrsind(const GRBGenConstr* xgenconstrs, int size);
    GRBConstr addConstr(const GRBLinExpr&  expr, char sense,
                        double lhs, double rhs, const string& cname);
    GRBConstr* addConstrs(const GRBLinExpr* expr, const char* sense,
                          const double* lhs, const double* rhs,
                          const string* name, int len);
    GRBQConstr addQConstr(const GRBQuadExpr&  expr, char sense,
                          const string& cname);
    double feasRelaxP(int type, bool minrelax, int vlen, int clen,
                    const GRBVar* vars, const double* lbpen,
                    const double* ubpen, const GRBConstr* constrs,
                    const double* rhspen);

  public:

    GRBModel(const GRBEnv& env);
    GRBModel(const GRBEnv& env, const string& filename);
    GRBModel(const GRBModel& xmodel);

    ~GRBModel();

    void read(const string& filename);
    void write(const string& filename);

    void sync();

    GRBModel relax();
    GRBModel fixedModel();
    GRBModel presolve();
    GRBModel feasibility();
    GRBModel linearize();

    double feasRelax(int relaxobjtype, bool minrelax, bool vrelax, bool crelax);
    double feasRelax(int relaxobjtype, bool minrelax, int vlen,
                   const GRBVar* vars, const double* lbpen,
                   const double* ubpen, int clen, const GRBConstr* constrs,
                   const double* rhspen);

    void update();
    void optimize();
    void optimizeasync();
    void computeIIS();
    void tune();
    void reset();
    void check();
    void terminate();

    void getTuneResult(int i);

    GRBQuadExpr getObjective() const;
    int  getPWLObj(GRBVar v, double *x, double *y) const;
    void setObjective(GRBLinExpr obje, int sense=0);
    void setObjective(GRBQuadExpr obje, int sense=0);
    void setPWLObj(GRBVar v, int points, double *x, double *y);

    GRBVar getVar(int i) const;
    GRBVar* getVars() const;
    GRBVar getVarByName(const string& name);
    GRBConstr getConstr(int i) const;
    GRBConstr* getConstrs() const;
    GRBConstr getConstrByName(const string& name);
    GRBSOS* getSOSs() const;
    GRBQConstr* getQConstrs() const;
    GRBGenConstr* getGenConstrs() const;

    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  string vname="");
    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  int nonzeros, const GRBConstr* xconstrs,
                  const double* coeffs=NULL, string name="");
    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  const GRBColumn& col, string name="");
    GRBVar* addVars(int cnt, char type=GRB_CONTINUOUS);
    GRBVar* addVars(const double* lb, const double* ub,
                    const double* obj, const char* type,
                    const string* name, int len);
    GRBVar* addVars(const double* lb, const double *ub,
                    const double* obj, const char* type,
                    const string* name, const GRBColumn*
                    col, int len);

    GRBConstr addConstr(const GRBLinExpr& expr1, char sense,
                        const GRBLinExpr& expr2,
                        string name="");
    GRBConstr addConstr(const GRBLinExpr& expr, char sense, GRBVar v,
                        string name="");
    GRBConstr addConstr(GRBVar v1, char sense, GRBVar v2,
                        string name="");
    GRBConstr addConstr(GRBVar v, char sense, double rhs,
                        string name="");
    GRBConstr addConstr(const GRBLinExpr& expr, char sense, double rhs,
                        string name="");
    GRBConstr addConstr(const GRBTempConstr& tc, string name="");
    GRBConstr addRange(const GRBLinExpr& expr, double lower, double upper,
                       string name="");
    GRBConstr* addConstrs(int cnt);
    GRBConstr* addConstrs(const GRBLinExpr* expr, const char* sense,
                          const double* rhs, const string* name,
                          int len);
    GRBConstr* addRanges(const GRBLinExpr* expr, const double* lower,
                         const double* upper, const string* name,
                         int len);
    GRBSOS addSOS(const GRBVar* xvars, const double* weight, int len, int type);
    GRBQConstr addQConstr(const GRBQuadExpr& expr1, char sense,
                          const GRBQuadExpr& expr2,
                          string name="");
    GRBQConstr addQConstr(const GRBTempConstr& tc, string name="");
    GRBQConstr addQConstr(const GRBQuadExpr&  expr, char sense, double rhs,
                          string name="");
    GRBGenConstr addGenConstrMax(GRBVar resvar, const GRBVar* xvars,
                                 int len, double constant=-GRB_INFINITY, string name="");
    GRBGenConstr addGenConstrMin(GRBVar resvar, const GRBVar* xvars,
                                 int len, double constant=GRB_INFINITY, string name="");
    GRBGenConstr addGenConstrAbs(GRBVar resvar, GRBVar argvar,
                                 string name="");
    GRBGenConstr addGenConstrAnd(GRBVar resvar, const GRBVar* xvars,
                                 int len, string name="");
    GRBGenConstr addGenConstrOr(GRBVar resvar, const GRBVar* xvars,
                                int len, string name="");
    GRBGenConstr addGenConstrIndicator(GRBVar binvar, int binval,
                                       const GRBLinExpr& expr, char sense, double rhs,
                                       string name="");
    GRBGenConstr addGenConstrIndicator(GRBVar binvar, int binval,
                                       const GRBTempConstr& constr,
                                       string name="");

    void remove(GRBVar v);
    void remove(GRBConstr c);
    void remove(GRBSOS xsos);
    void remove(GRBQConstr xqconstr);
    void remove(GRBGenConstr xgenconstr);

    void chgCoeff(GRBConstr c, GRBVar v, double val);
    void chgCoeffs(const GRBConstr* xconstrs, const GRBVar* xvars,
                   const double* val, int len);
    void chgCoeffs(const GRBConstr* xconstrs, const GRBVar* xvars,
                   const double* val, size_t len);
    double getCoeff(GRBConstr c, GRBVar v) const;
    GRBColumn getCol(GRBVar v);
    GRBLinExpr getRow(GRBConstr c);
    int getSOS(GRBSOS xsos, GRBVar* xvars, double* weight, int* typeP);
    void getGenConstrMax(GRBGenConstr genc, GRBVar* resvarP, GRBVar* xvars,
                         int* lenP, double* constantP);
    void getGenConstrMin(GRBGenConstr genc, GRBVar* resvarP, GRBVar* xvars,
                         int* lenP, double* constantP);
    void getGenConstrAbs(GRBGenConstr genc, GRBVar* resvarP, GRBVar* argvarP);
    void getGenConstrAnd(GRBGenConstr genc, GRBVar* resvarP, GRBVar* xvars,
                         int* lenP);
    void getGenConstrOr(GRBGenConstr genc, GRBVar* resvarP, GRBVar* xvars,
                        int* lenP);
    void getGenConstrIndicator(GRBGenConstr genc, GRBVar* binvarP, int* binvalP,
                               GRBLinExpr* exprP, char* senseP, double* rhsP);

    GRBQuadExpr getQCRow(GRBQConstr c);
    GRBEnv getEnv();
    GRBEnv getConcurrentEnv(int num);
    void discardConcurrentEnvs();

    // Parameters

    int    get(GRB_IntParam param) const;
    double get(GRB_DoubleParam param) const;
    string get(GRB_StringParam param) const;

    void set(GRB_IntParam param, int val);
    void set(GRB_DoubleParam param, double val);
    void set(GRB_StringParam param, const string& val);
    void set(const string& param, const string& val);

    // Attributes

    int    get(GRB_IntAttr attr) const;
    double get(GRB_DoubleAttr attr) const;
    string get(GRB_StringAttr attr) const;

    void set(GRB_IntAttr attr, int val);
    void set(GRB_DoubleAttr attr, double val);
    void set(GRB_StringAttr attr, const string& val);

    int*    get(GRB_IntAttr    attr, const GRBVar* xvars, int len);
    char*   get(GRB_CharAttr   attr, const GRBVar* xvars, int len);
    double* get(GRB_DoubleAttr attr, const GRBVar* xvars, int len);
    string* get(GRB_StringAttr attr, const GRBVar* xvars, int len);

    int*    get(GRB_IntAttr    attr, const GRBConstr* xconstrs, int len);
    char*   get(GRB_CharAttr   attr, const GRBConstr* xconstrs, int len);
    double* get(GRB_DoubleAttr attr, const GRBConstr* xconstrs, int len);
    string* get(GRB_StringAttr attr, const GRBConstr* xconstrs, int len);

    int*    get(GRB_IntAttr    attr, const GRBQConstr* xqconstrs, int len);
    char*   get(GRB_CharAttr   attr, const GRBQConstr* xqconstrs, int len);
    double* get(GRB_DoubleAttr attr, const GRBQConstr* xqconstrs, int len);
    string* get(GRB_StringAttr attr, const GRBQConstr* xqconstrs, int len);

    int*    get(GRB_IntAttr    attr, const GRBGenConstr* xgenconstrs, int len);
    string* get(GRB_StringAttr attr, const GRBGenConstr* xgenconstrs, int len);

    void set(GRB_IntAttr    attr, const GRBVar* xvars,
             const int*    val, int len);
    void set(GRB_CharAttr   attr, const GRBVar* xvars,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBVar* xvars,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBVar* xvars,
             const string* val, int len);

    void set(GRB_IntAttr    attr, const GRBConstr* xconstrs,
             const int*    val, int len);
    void set(GRB_CharAttr   attr, const GRBConstr* xconstrs,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBConstr* xconstrs,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBConstr* xconstrs,
             const string* val, int len);

    void set(GRB_CharAttr   attr, const GRBQConstr* xconstrs,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBQConstr* xconstrs,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBQConstr* xconstrs,
             const string* val, int len);

    void set(GRB_StringAttr attr, const GRBGenConstr* xqconstrs,
             const string* val, int len);

    void setCallback(GRBCallback* xcb);
};
#endif
