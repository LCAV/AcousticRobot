(TeX-add-style-hook
 "report"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("mcode" "framed")))
   (TeX-run-style-hooks
    "ex1"
    "ex2"
    "ex3"
    "mcode")))

