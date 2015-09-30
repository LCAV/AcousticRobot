\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{pset}[2013/10/25 Kiki's awesome problem set class]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass[a4paper,twoside]{article}

% font/input encoding
\RequirePackage[T1]{fontenc} % Use 8-bit encoding that has 256 glyphs
\RequirePackage[utf8]{inputenc}
\RequirePackage[sc]{mathpazo} % Use the Palatino font

% etoolbox for definitions
\RequirePackage{etoolbox}


\linespread{1.05} % Line spacing - Palatino needs more space between lines
\RequirePackage{microtype} % Slightly tweak font spacing for aesthetics

\RequirePackage[hmarginratio=1:1,top=32mm,columnsep=20pt]{geometry} % Document margins
\RequirePackage[hang, small,labelfont=bf,up,textfont=it,up]{caption} % Custom captions under/above floats in tables or figures
\RequirePackage{float} % Required for tables and figures in the multi-column environment - they need to be placed in specific locations with the [H] (e.g. \begin{table}[H])
\RequirePackage{hyperref} % For hyperlinks in the PDF

\RequirePackage{subcaption}
\RequirePackage{paralist} % Used for the compactitem environment which makes bullet points with less space between them

\RequirePackage{abstract} % Allows abstract customization
\renewcommand{\abstractnamefont}{\normalfont\bfseries} % Set the "Abstract" text to bold
\renewcommand{\abstracttextfont}{\normalfont\small\itshape} % Set the abstract itself to small italic text
\RequirePackage{graphicx}
\RequirePackage{booktabs} % Horizontal rules in tables


\RequirePackage{titlesec} % Allows customization of titles
\renewcommand\thesection{\Roman{section}} % Roman numerals for the sections
\renewcommand\thesubsection{\Roman{subsection}} % Roman numerals for subsections
\titleformat{\section}[block]{\large\scshape\centering}{\thesection.}{1em}{} % Change the look of the section titles
\titleformat{\subsection}[block]{\large}{\thesubsection.}{1em}{} % Change the look of the section titles

\RequirePackage{fancyhdr} % Headers and footers
\pagestyle{fancy} % All pages have headers and footers
\fancyhead{} % Blank out the default header
\fancyfoot{} % Blank out the default footer
\fancyhead[C]{\@header} % Custom header text
\fancyfoot[RO,LE]{\thepage} % Custom footer text

\thispagestyle{fancy} % All pages have headers and footers

\setlength{\headsep}{15pt}

\newcommand{\institution}[1]{\def\@institution{#1}}
\newcommand{\email}[1]{\def\@email{#1}}
\newcommand{\header}[1]{\def\@header{#1}}
\newcommand{\@header}{}


%----------------------------------------------------------------------------------------
%	TITLE SECTION
%----------------------------------------------------------------------------------------
\renewcommand{\maketitle}{
    {\centering
    \fontsize{24pt}{10pt}\selectfont\textbf{\@title } \\[5mm]% Article title
	\large
    \textsc{\@author}\\[2mm] % Your name
    \ifdef{\@institution}{\normalsize \@institution \\[0.5mm]}{} % Your institution
    \ifdef{\@email}{\normalsize \@email \\}{}%
}
    \vspace{-0.5em}
}
