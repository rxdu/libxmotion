namespace matplot
{
  template<typename Vec_t>
  inline int vec_dim(const Vec_t& v)
  { return num_rows(v); }

  template<typename Mat_t>
  inline int mat_dim(const Mat_t& A, int dim)
  {
    if(dim == 1) return num_rows(A);
    if(dim == 2) return num_cols(A);
    return 0;
  }
}
