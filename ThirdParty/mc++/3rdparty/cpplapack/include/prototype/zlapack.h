extern "C" {
  // Solve Linear Equations  A * x = b
  /* for General Matrix */
  void zgesv_( const long &N, const long &nrhs, std::complex<double> *a,
               const long &lda, long *ipiv, std::complex<double> *b,
               const long &ldb, long &info );
  /* for General Band Matrix */
  void zgbsv_( const long &N, const long &KL, const long &KU,
               const long &nrhs, std::complex<double> *ab, const long &ldab,
               long *ipiv, std::complex<double> *b, const long &ldb,
               long &info );
  /* for Tridiagonal Matrix */
  void zgtsv_( const long &N, const long &nrhs, std::complex<double> *dl,
               std::complex<double> *d, std::complex<double> *du,
               std::complex<double> *b, const long &ldb, long &info );
  /* for Hermitian Positive Definite Matrix */
  void zposv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *a, const long &lda, std::complex<double> *b,
               const long &ldb, long &info );
  /* for Hermitian Positive Definite (Packed Storage) Matrix */
  void zppsv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *ap, std::complex<double> *b, const long &ldb,
               long &info );
  /* for Hermitian Positive Definite Band Matrix */
  void zpbsv_( const char &uplo, const long &N, const long &kd,
               const long &nrhs, std::complex<double> *ab, const long &ldab,
               std::complex<double> *b, const long &ldb, long &info );
  /* for Hermitian Positive Definite Tridiagonal Matrix */
  void zptsv_( const long &N, const long &nrhs, double *d,
               std::complex<double> *e, std::complex<double> *b, const long &ldb,
               long &info );
  /* for Hermitian Indefinite Matrix */
  void zhesv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *a, const long &lda, long *ipiv,
               std::complex<double> *b, const long &ldb, std::complex<double> *work,
               const long &lwork, long &info );
  /* for Symmetric Indefinite Matrix */
  void zsysv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *a, const long &lda, long *ipiv,
               std::complex<double> *b, const long &ldb, std::complex<double> *work,
               const long &lwork, long &info );
  /* for Hermitian Indefinite (Packed Storage) Matrix */
  void zhpsv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *ap, long *ipiv, std::complex<double> *b,
               const long &ldb, long &info );
  /* for Symmetric Indefinite (Packed Storage) Matrix */
  void zspsv_( const char &uplo, const long &N, const long &nrhs,
               std::complex<double> *ap, long *ipiv, std::complex<double> *b,
               const long &ldb, long &info );

  // Linear Least Square Problems
  // Solve Overdetermined or Underdetermined Linear Equations
  /* Using Orthogonal Factorization, Assuming Full Rank */
  void zgels_( const char &trans, const long &M, const long &N,
               const long &nrhs, std::complex<double> *a, const long &lda,
               std::complex<double> *b, const long &ldb,
               std::complex<double> *work, const long &lwork, long &info );
  /* Compute Minimum-Norm Solution using Orthogonal Factorization */
  void zgelsy_( const long &M, const long &N, const long &nrhs,
                std::complex<double> *a, const long &lda, std::complex<double> *b,
                const long &ldb, long *jpvt, const double &rcond,
                long &rank, std::complex<double> *work, const long &lwork,
                double *rwork, long &info );
  /* Compulte Minimum-Norm Solution using Singular Value Decomposition */
  void zgelss_( const long &M, const long &N, const long &nrhs,
                std::complex<double> *a, const long &lda, std::complex<double> *b,
                const long &ldb, double *s, const double &rcond,
                long &rank, std::complex<double> *work, const long &lwork,
                double *rwork, long &info );
  /* Solve Linear Equality-Constrained Least Squares (LSE) Problem */
  void zgglse_( const long &M, const long &N, const long &p,
                std::complex<double> *a, const long &lda, std::complex<double> *b,
                const long &ldb, std::complex<double> *c, std::complex<double> *d,
                std::complex<double> *x, std::complex<double> *work,
                const long &lwork, long &info );
  /* Solve Gauss-Markov Linear Model (GLM) Problem */
  void zggglm_( const long &N, const long &M, const long &p,
                std::complex<double> *a, const long &lda, std::complex<double> *b,
                const long &ldb, std::complex<double> *d, std::complex<double> *x,
                std::complex<double> *y, std::complex<double> *work,
                const long &lwork, long &info );

  // Standard Eigenvalue and Singular Value Problems
  // Divide and Conquer routines are more fast, but need more memory.
  /* Eigenvalues/Eigenvectors for General Matrix */
  void zgeev_( const char &jobvl, const char &jobvr, const long &N,
               std::complex<double> *a, const long &lda, std::complex<double> *w,
               std::complex<double> *vl, const long &ldvl, std::complex<double> *vr,
               const long &ldvr, std::complex<double> *work, const long &lwork,
               double *rwork, long &info );
  /* Eigenvalues/Eigenvectors for Hermitian Matrix */
  void zheev_( const char &jobz, const char &uplo, const long &N,
               std::complex<double> *a, const long &lda, double *w,
               std::complex<double> *work, const long &lwork, double *rwork,
               long &info );
  void zheevd_( const char &jobz, const char &uplo, const long &N,
                std::complex<double> *a, const long &lda, double *w,
                std::complex<double> *work, const long &lwork, double *rwork,
                const long &lrwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Eigenvalues/Eigenvectors for Hermitian (Packed Storage) Matrix */
  void zhpev_( const char &jobz, const char &uplo, const long &N,
               std::complex<double> *ap, double *w, std::complex<double> *z,
               const long &ldz, std::complex<double> *work, double *rwork,
               long &info );
  void zhpevd_( const char &jobz, const char &uplo, const long &N,
                std::complex<double> *ap, double *w, std::complex<double> *z,
                const long &ldz, std::complex<double> *work, const long &lwork,
                double *rwork, const long &lrwork, long *iwork,
                const long &liwork, long &info );  /* Divide and Conqure */
  /* Eigenvalues/Eigenvectors for Hermitian Band Matrix */
  void zhbev_( const char &jobz, const char &uplo, const long &N,
               const long &kd, std::complex<double> *ab, const long &ldab,
               double *w, std::complex<double> *z, const long &ldz,
               std::complex<double> *work, double *rwork, long &info );
  void zhbevd_( const char &jobz, const char &uplo, const long &N,
                const long &kd, std::complex<double> *ab, const long &ldab,
                double *w, std::complex<double> *z, const long &ldz,
                std::complex<double> *work, const long &lwork, double *rwork,
                const long &lrwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Schur Factorization for General Matrix */
  void zgees_( const char &jobvs, const char &sort,
               bool (*select)( std::complex<double> *, std::complex<double> * ),
               const long &N, std::complex<double> *a, const long &lda,
               long &sdim, std::complex<double> *w, std::complex<double> *vs,
               const long &ldvs, std::complex<double> *work, const long &lwork,
               double *rwork, bool *bwork, long &info );
  /* Singular Value Decomposition for General Matrix */
  void zgesvd_( const char &jobu, const char &jobvt, const long &M,
                const long &N, std::complex<double> *a, const long &lda,
                double *s, std::complex<double> *u, const long &ldu,
                std::complex<double> *vt, const long &ldvt,
                std::complex<double> *work, const long &lwork, double *rwork,
                long &info );
  void zgesdd_( const char &jobz, const long &M, const long &N,
                std::complex<double> *a, const long &lda, double *s,
                std::complex<double> *u, const long &ldu, std::complex<double> *vt,
                const long &ldvt, std::complex<double> *work, const long &lwork,
                double *rwork, long *iwork,
                long &info );  /* Divide and Conqure */

  // Generalized Eigenvalue and Sigular Value Problems
  /* Generalized Eigenvalues/Eigenvectors for General Matrix */
  void zggev_( const char &jobvl, const char &jobvr, const long &N,
               std::complex<double> *a, const long &lda, std::complex<double> *b,
               const long &ldb, std::complex<double> *alpha,
               std::complex<double> *beta, std::complex<double> *vl,
               const long &ldvl, std::complex<double> *vr, const long &ldvr,
               std::complex<double> *work, const long &lwork, double *rwork,
               long &info );
  /* Generalized Eigenvalues/Eigenvectors
     for Hermitian-definite Matrix */
  void zhegv_( const long &itype, const char &jobz, const char &uplo,
               const long &N, std::complex<double> *a, const long &lda,
               std::complex<double> *b, const long &ldb, double *w,
               std::complex<double> *work, const long &lwork, double *rwork,
               long &info );
  void zhegvd_( const long &itype, const char &jobz, const char &uplo,
                const long &N, std::complex<double> *a, const long &lda,
                std::complex<double> *b, const long &ldb, double *w,
                std::complex<double> *work, const long &lwork, double *rwork,
                const long &lrwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Eigenvalues/Eigenvectors
     for Hermitian-definite (Packed Storage) Matrix */
  void zhpgv_( const long &itype, const char &jobz, const char &uplo,
               const long &N, std::complex<double> *ap, std::complex<double> *bp,
               double *w, std::complex<double> *z, const long &ldz,
               std::complex<double> *work, double *rwork, long &info );
  void zhpgvd_( const long &itype, const char &jobz, const char &uplo,
                const long &N, std::complex<double> *ap, std::complex<double> *bp,
                double *w, std::complex<double> *z, const long &ldz,
                std::complex<double> *work, const long &lwork, double *rwork,
                const long &lrwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Eigenvalues/Eigenvectors
     for Hermitian-definite Band Matrix */
  void zhbgv_( const char &jobz, const char &uplo, const long &N,
               const long &ka, const long &kb, std::complex<double> *ab,
               const long &ldab, std::complex<double> *bb, const long &ldbb,
               double *w, std::complex<double> *z, const long &ldz,
               std::complex<double> *work, double *rwork, long &info );
  void zhbgvd_( const char &jobz, const char &uplo, const long &N,
                const long &ka, const long &kb, std::complex<double> *ab,
                const long &ldab, std::complex<double> *bb, const long &ldbb,
                double *w, std::complex<double> *z, const long &ldz,
                std::complex<double> *work, const long &lwork, double *rwork,
                const long &lrwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Schur Factrization for General Matrix */
  void zgges_( const char &jobvsl, const char &jobvsr, const char &sort,
               bool (*delctg)( std::complex<double> *, std::complex<double> * ),
               const long &N, std::complex<double> *a, const long &lda,
               std::complex<double> *b, const long &ldb, long &sdim,
               std::complex<double> *alpha, std::complex<double> *beta,
               std::complex<double> *vsl, const long &ldvsl,
               std::complex<double> *vsr, const long &ldvsr,
               std::complex<double> *work, const long &lwork, double *rwork,
               bool *bwork, long &info );
  /* Generailized Singular Value Decomposition for General Matrix */
  void zggsvd_( const char &jobu, const char &jobv, const char &jobq,
                const long &M, const long &N, const long &p, long &k,
                long &L, std::complex<double> *a, const long &lda,
                std::complex<double> *b, const long &ldb, double *alpha,
                double *beta, std::complex<double> *u, const long &ldu,
                std::complex<double> *v, const long &ldv, std::complex<double> *q,
                const long &ldq, std::complex<double> *work, double *rwork,
                long *iwork, long &info );
}
