namespace matplot
{
  template<typename Vec_t>
  inline int vec_dim(const Vec_t& v)
  { return v.size(); }

  template<typename Mat_t>
  inline int mat_dim(const Mat_t& A, int dim)
  {
    if(dim == 1) return A.size1();
    if(dim == 2) return A.size2();
    return 0;
  }
}
