extern "C" {
  // Level 1 BLAS
  /* Vector rotation: x := c*x[i] + s*y[i], y[i] := - s*x[i] + c*y[i] */
  void drot_( const long &N, double *x, const long &incx, double *y,
              const long &incy, const double &c, const double &s );
  /* x <--> y */
  void dswap_( const long &N, double *x, const long &incx, double *y,
               const long &incy );
  /* x := alpha * x */
  void dscal_( const long &N, const double &alpha, double *x,
               const long &incx );
  /* y := x */
  void dcopy_( const long &N, const double *x, const long &incx,
               double *y, const long &incy );
  /* y := alpha * x + y */
  void daxpy_( const long &N, const double &alpha, const double *x,
               const long &incx, double *y, const long &incy );
  /* return x^T y */
  double ddot_( const long &N, const double *x, const long &incx,
                const double *y, const long &incy );
  /* return N2 norm */
  double dnrm2_( const long &N, const double *x, const long &incx );
  /* return sum of abs(x[i]) */
  double dasum_( const long &N, const double *x, const long &incx );
  /* return the index of element having the largest absolute value */
  long idamax_( const long &N, const double *x, const long &incx );

  // Level 2 BLAS
  /* y := alpha * A * x + beta * y for General M by N Matrix */
  void dgemv_( const char &trans, const long &M, const long &N,
               const double &alpha, const double *a, const long &lda,
               const double *x, const long &incx, const double &beta,
               double *y, const long &incy );
  /* y := alpha * A * x + beta * y for General M by N Band Matrix */
  void dgbmv_( const char &trans, const long &M, const long &N,
               const long &KL, const long &KU, const double &alpha,
               const double *a, const long &lda, const double *x,
               const long &incx, const double &beta, double *y,
               const long &incy );
  /* y := alpha * A * x + beta * y for Symmetric Matrix */
  void dsymv_( const char &uplo, const long &N, const double &alpha,
               const double *a, const long &lda, const double *x,
               const long &incx, const double &beta, double *y,
               const long &incy );
  /* y := alpha * A * x + beta * y for Symmetric Band Matrix */
  void dsbmv_( const char &uplo, const long &N, const long &k,
               const double &alpha, const double *a, const long &lda,
               const double *x, const long &incx, const double &beta,
               double *y, const long &incy );
  /* y := alpha * A * x + beta * y for Symmetric Packed Matrix */
  void dspmv_( const char &uplo, const long &N, const double &alpha,
               const double *ap, const double *x, const long &incx,
               const double &beta, double *y, const long &incy );

  /* x := A * x for Triangular Matrix */
  void dtrmv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const double *a, const long &lda,
               double *x, const long &incx );
  /* x := A * x for Triangular (Banded Storage) Matrix */
  void dtbmv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const long &k, const double *a,
               const long &lda, double *x, const long &incx );
  /* x := A * x for Triangular (Packed Storage) Matrix */
  void dtpmv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const double *ap, double *x,
               const long &incx );

  /* Solve A * x = b for Triangular Matrix */
  void dtrsv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const double *a, const long &lda, double *x,
               const long &incx );
  /* Solve A * x = b for Triangular (Banded Storage) Matrix */
  void dtbsv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const long &k, const double *a,
               const long &lda, double *x, const long &incx );
  /* Solve A * x = b for Triangular (Packed Storage) Matrix */
  void dtpsv_( const char &uplo, const char &trans, const char &diag,
               const long &N, const double *ap, double *x,
               const long &incx );

  /* A := alpha * x * y' + A  (A: M by N Matrix) */
  void dger_( const long &M, const long &N, const double &alpha,
              const double *x, const long &incx, const double *y,
              const long &incy, double *a, const long &lda );
  /* A := alpha * x * x' + A  (A: Symmetric N by N Matrix) */
  void dsyr_( const char &uplo, const long &N, const double &alpha,
              const double *x, const long &incx, double *a,
              const long &lda );
  /* A := alpha * x * x' + A
     (A: Symmetric N by N Packed Storage Matrix) */
  void dspr_( const char &uplo, const long &N, const double &alpha,
              const double *x, const long &incx, double *ap );
  /* A := alpha * x * y' + alpha * y * x' + A
     (A: Symmetric N by N Matrix) */
  void dsyr2_( const char &uplo, const long &N, const double &alpha,
               const double *x, const long &incx, const double *y,
               const long &incy, double *a, const long &lda );
  /* A := alpha * x * y' + alpha * y * x' + A
     (A: Symmetric N by N Packed Storage Matrix) */
  void dspr2_( const char &uplo, const long &N, const double &alpha,
               const double *x, const long &incx, const double *y,
               const long &incy, double *ap );

  // Level 3 BLAS
  /* C := alpha * A * B + beta * C  (C: M by N Matrix) */
  void dgemm_( const char &transa, const char &transb, const long &M,
               const long &N, const long &k, const double &alpha,
               const double *a, const long &lda, const double *b,
               const long &ldb, const double &beta, double *c,
               const long &ldc );
  /* C := alpha * A * B + beta * C
     (A: Symmetric Matrix,  B, C: M by N Matrices) */
  void dsymm_( const char &side, const char &uplo, const long &M,
               const long &N, const double &alpha, const double *a,
               const long &lda, const double *b, const long &ldb,
               const double &beta, double *c, const long &ldc );
  /* C:= alpha * A * A' + beta * C
     (A: N by k Matrix,  C: Symmetric Matrix) */
  void dsyrk_( const char &uplo, const char &trans, const long &N,
               const long &k, const double &alpha, const double *a,
               const long &lda, const double &beta, double *c,
               const long &ldc );
  /* C := alpha * A * B' + alpha * B * A' + beta * C
     (A, B: N by k Marices,  C: Symmetric Matrix ) */
  void dsyr2k_( const char &uplo, const char &trans, const long &N,
                const long &k, const double &alpha, const double *a,
                const long &lda, const double *b, const long &ldb,
                const double &beta, double *c, const long &ldc );
  /* B := alpha * A * B  (A: Triangular Matrix,  B: M by N Matrix) */
  void dtrmm_( const char &side, const char &uplo, const char &transa,
               const char &diag, const long &M, const long &N,
               const double &alpha, const double *a, const long &lda,
               double *b, const long &ldb );
  /* Solve A * X = alpha * B
     (A: Triangular Matrix,  X, B: M by N Matrices) */
  void dtrsm_( const char &side, const char &uplo, const char &transa,
               const char &diag, const long &M, const long &N,
               const double &alpha, const double *a, const long &lda,
               double *b, const long &ldb );
}
