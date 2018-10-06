extern "C" {
  // Solve Linear Equations  A * x = b
  /* for General Matrix */
  void dgesv_( const long &N, const long &nrhs, double *a, const long &lda,
               long *ipiv, double *b, const long &ldb, long &info );
  /* for General Band Matrix */
  void dgbsv_( const long &N, const long &KL, const long &KU,
               const long &nrhs, double *ab, const long &ldab,
               long *ipiv, double *b, const long &ldb, long &info );
  /* for Tridiagonal Matrix */
  void dgtsv_( const long &N, const long &nrhs, double *dl, double *d,
               double *du, double *b, const long &ldb, long &info );
  /* for Symmetric Positive Definite Matrix */
  void dposv_( const char &uplo, const long &N, const long &nrhs,
               double *a, const long &lda, double *b, const long &ldb,
               long &info );
  /* for Symmetric Positive Definite (Packed Storage) Matrix */
  void dppsv_( const char &uplo, const long &N, const long &nrhs,
               double *ap, double *b, const long &ldb, long &info );
  /* for Symmetric Positive Definite Band Matrix */
  void dpbsv_( const char &uplo, const long &N, const long &kd,
               const long &nrhs, double *ab, const long &ldab,
               double *b, const long &ldb, long &info );
  /* for Symmetric Positive Definite Tridiagonal Matrix */
  void dptsv_( const long &N, const long &nrhs, double *d, double *e,
               double *b, const long &ldb, long &info );
  /* for Symmetric Indefinite Matrix */
  void dsysv_( const char &uplo, const long &N, const long &nrhs,
               double *a, const long &lda, long *ipiv, double *b,
               const long &ldb, double *work, const long &lwork,
               long &info );
  /* for Symmetric Indefinite (Packed Storage) Matrix */
  void dspsv_( const char &uplo, const long &N, const long &nrhs,
               double *ap, long *ipiv, double *b, const long &ldb,
               long &info );

  // Linear Least Square Problems
  // Solve Overdetermined or Underdetermined Linear Equations
  /* Using Orthogonal Factorization, Assuming Full Rank */
  void dgels_( const char &trans, const long &M, const long &N,
               const long &nrhs, double *a, const long &lda,
               double *b, const long &ldb,
               double *work, const long &lwork, long &info );
  /* Compute Minimum-Norm Solution using Orthogonal Factorization */
  void dgelsy_( const long &M, const long &N, const long &nrhs,
                double *a, const long &lda, double *b, const long &ldb,
                long *jpvt, const double &rcond, long &rank,
                double *work, const long &lwork, long &info );
  /* Compulte Minimum-Norm Solution using Singular Value Decomposition */
  void dgelss_( const long &M, const long &N, const long &nrhs,
                double *a, const long &lda, double *b, const long &ldb,
                double *s, const double &rcond, long &rank,
                double *work, const long &lwork, long &info );
  /* Solve Linear Equality-Constrained Least Squares (LSE) Problem */
  void dgglse_( const long &M, const long &N, const long &p, double *a,
                const long &lda, double *b, const long &ldb,
                double *c, double *d, double *x, double *work,
                const long &lwork, long &info );
  /* Solve Gauss-Markov Linear Model (GLM) Problem */
  void dggglm_( const long &N, const long &M, const long &p,
                double *a, const long &lda, double *b, const long &ldb,
                double *d, double *x, double *y,
                double *work, const long &lwork, long &info );

  // Standard Eigenvalue and Singular Value Problems
  // Divide and Conquer routines are more fast, but need more memory.
  /* Eigenvalues/Eigenvectors for General Matrix */
  void dgeev_( const char &jobvl, const char &jobvr, const long &N,
               double *a, const long &lda, double *wr, double *wi,
               double *vl, const long &ldvl, double *vr, const long &ldvr,
               double *work, const long &lwork, long &info );
  /* Eigenvalues/Eigenvectors for Symmetric Matrix */
  void dsyev_( const char &jobz, const char &uplo, const long &N,
               double *a, const long &lda, double *w, double *work,
               const long &lwork, long &info );
  void dsyevd_( const char &jobz, const char &uplo, const long &N,
                double *a, const long &lda, double *w, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Eigenvalues/Eigenvectors for Symmetric (Packed Storage) Matrix */
  void dspev_( const char &jobz, const char &uplo, const long &N,
               double *ap, double *w, double *z, const long &ldz,
               double *work, long &info );
  void dspevd_( const char &jobz, const char &uplo, const long &N,
                double *ap, double *w, double *z, const long &ldz,
                double *work, const long &lwork, long *iwork,
                const long &liwork, long &info );  /* Divide and Conqure */
  /* Eigenvalues/Eigenvectors for Symmetric Band Matrix */
  void dsbev_( const char &jobz, const char &uplo, const long &N,
               const long &kd, double *ab, const long &ldab, double *w,
               double *z, const long &ldz, double *work, long &info );
  void dsbevd_( const char &jobz, const char &uplo, const long &N,
                const long &kd, double *ab, const long &ldab, double *w,
                double *z, const long &ldz, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Eigenvalues/Eigenvectors for Symmetric Tridiagonal Matrix */
  void dstev_( const char &jobz, const long &N, double *d, double *e,
               double *z, const long &ldz, double *work, long &info );
  void dstevd_( const char &jobz, const long &N, double *d, double *e,
                double *z, const long &ldz, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Schur Factorization for General Matrix */
  void dgees_( const char &jobvs, const char &sort,
               bool (*select)( double *, double * ),
               const long &N, double *a, const long &lda, long &sdim,
               double *wr, double *wi, double *vs, const long &ldvs,
               double *work, const long &lwork, bool *bwork,
               long &info );
  /* Singular Value Decomposition for General Matrix */
  void dgesvd_( const char &jobu, const char &jobvt, const long &M,
                const long &N, double *a, const long &lda, double *s,
                double *u, const long &ldu, double *vt, const long &ldvt,
                double *work, const long &lwork, long &info );
  void dgesdd_( const char &jobz, const long &M, const long &N, double *a,
                const long &lda, double *s, double *u, const long &ldu,
                double *vt, const long &ldvt, double *work,
                const long &lwork, long *iwork,
                long &info );  /* Divide and Conqure */

  // Generalized Eigenvalue and Sigular Value Problems
  /* Generalized Eigenvalues/Eigenvectors for General Matrix */
  void dggev_( const char &jobvl, const char &jobvr, const long &N,
               double *a, const long &lda, double *b, const long &ldb,
               double *alphar, double *alphai, double *beta,
               double *vl, const long &ldvl, double *vr, const long &ldvr,
               double *work, const long &lwork, long &info );
  /* Generalized Eigenvalues/Eigenvectors
     for Symmetric-definite Matrix */
  void dsygv_( const long &itype, const char &jobz, const char &uplo,
               const long &N, double *a, const long &lda, double *b,
               const long &ldb, double *w, double *work, const long &lwork,
               long &info );
  void dsygvd_( const long &itype, const char &jobz, const char &uplo,
                const long &N, double *a, const long &lda, double *b,
                const long &ldb, double *w, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Eigenvalues/Eigenvectors
     for Symmetric-definite (Packed Storage) Matrix */
  void dspgv_( const long &itype, const char &jobz, const char &uplo,
               const long &N, double *ap, double *bp, double *w,
               double *z, const long &ldz, double *work, long &info );
  void dspgvd_( const long &itype, const char &jobz, const char &uplo,
                const long &N, double *ap, double *bp, double *w,
                double *z, const long &ldz, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Eigenvalues/Eigenvectors
     Symmetric-definite Band Matrix */
  void dsbgv_( const char &jobz, const char &uplo, const long &N,
               const long &ka, const long &kb, double *ab, const long &ldab,
               double *bb, const long &ldbb, double *w, double *z,
               const long &ldz, double *work, long &info );
  void dsbgvd_( const char &jobz, const char &uplo, const long &N,
                const long &ka, const long &kb, double *ab,
                const long &ldab, double *bb, const long &ldbb, double *w,
                double *z, const long &ldz, double *work,
                const long &lwork, long *iwork, const long &liwork,
                long &info );  /* Divide and Conqure */
  /* Generalized Schur Factrization for General Matrix */
  void dgges_( const char &jobvsl, const char &jobvsr, const char &sort,
               long (*delctg)( double *, double *, double * ),
               const long &N, double *a, const long &lda, double *b,
               const long &ldb, long &sdim, double *alphar, double *alphai,
               double *beta, double *vsl, const long &ldvsl, double *vsr,
               const long &ldvsr, double *work, const long &lwork,
               bool *bwork, long &info );
  /* Generailized Singular Value Decomposition for General Matrix */
  void dggsvd_( const char &jobu, const char &jobv, const char &jobq,
                const long &M, const long &N, const long &p, long &k,
                long &L, double *a, const long &lda, double *b,
                const long &ldb, double *alpha, double *beta,
                double *u, const long &ldu, double *v, const long &ldv,
                double *q, const long &ldq, double *work, long *iwork,
                long &info );
}
