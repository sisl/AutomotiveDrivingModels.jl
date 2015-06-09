using Base.Test
using Lint

@lintpragma( "Ignore undefined module [Features]" )
@lintpragma( "Ignore use of undeclared variable [NA_ALIAS]" )
@lintpragma( "Ignore unstable type variable [AbstractFeature]" )

lintpkg("CarEM")

# using CarEM

