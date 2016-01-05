%==========================================
%
% class for report of Data analysis's practical works 
%
%==========================================

% Declare Class

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{flex}


%Load mother's class

%---- Recto seulement ----%
%\LoadClass[11pt,onecolumn,a4paper,oneside,final]{article}
%---- Recto verso ----%

\LoadClass[11pt,onecolumn,a4paper,twoside]{article}

%fundamental packages
\RequirePackage[utf8]{inputenc}
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}
\RequirePackage[english]{babel}
%\RequirePackage[british,UKenglish,USenglish,english,american]{babel}
\RequirePackage[framed]{mcode}
\RequirePackage{verbatim}  % Needed for the "comment" environment to make LaTeX comments
\RequirePackage[section]{placeins} % adds float barrier with each section
\RequirePackage{float}

%mathematics packages
\RequirePackage{array}
\RequirePackage{amsmath}
\RequirePackage{amssymb}
\RequirePackage[version=3]{mhchem}
\RequirePackage[retainorgcmds]{IEEEtrantools}

%Figures packages
\RequirePackage{graphicx}
\RequirePackage{wrapfig}
\RequirePackage{subcaption}
%\RequirePackage{subfigure}
%\RequirePackage[center]{subcaption}
%\RequirePackage[center]{subfigure}
\RequirePackage[font={small,it}]{caption}

%font-codes package
\RequirePackage{color}
\RequirePackage{comment}
\RequirePackage{hyperref}

% code listings
\RequirePackage{listings}
\RequirePackage{xcolor}
\lstset{
numbers=left, 
frame=lrtb,
numbersep=5pt,
frame=single,
breaklines=true,
postbreak=\raisebox{0ex}[0ex][0ex]{\ensuremath{\color{blue}\hookrightarrow\space}}
}
\renewcommand{\lstlistingname}{Code}

% package for using SCI formulas
\RequirePackage{siunitx}
% be weird and use apostrophes to separate your numbers like a weirdo
\sisetup{group-separator = {'}}
% KATIE: you can use this command in math mode or normal mode (it automatically goes into math mode) to enter a number followed by a unit, properly formatted
% you can also use abbreviations like \u{2}{\micro\second} and it figues it out
% it also handles spacing and such of your number
\renewcommand{\u}[2]{\ensuremath{\SI{#1}{#2}}}


% Paper size and Layout

\RequirePackage[dvips, 
	    a4paper, 
	    top			=1cm, 
	    bottom			=2cm, 
	    left			=2cm, 
	    right 			=2cm, 
	    headheight		=6mm, 
	    headsep		=10mm, 
	    marginparwidth	=2cm, 
	    marginparsep	=4mm, 
	    heightrounded, 
	    includehead]{geometry} 


%names and informations
\newcommand\subtitle[1]{\def\@subtitle{#1}}
\newcommand\teacher[1]{\def\@teacher{#1}}
\newcommand\footauthor[1]{\def\@footauthor{#1}}
\newcommand\semester[1]{\def\@semester{#1}}
\newcommand\cours[1]{\def\@cours{#1}}
\newcommand\email[1]{\def\@email{#1}}

%---- Tables ----%
\RequirePackage{tabularx}
\RequirePackage{booktabs} % in order to use midrule, toprule etc.
\RequirePackage{multirow}
\RequirePackage{multicol}

%----Newcommand----%

\newcommand{\figi}[4]{
   \begin{figure}[!ht]
   \begin{center}
   \includegraphics[width=#4]{fig/#1}
   \end{center}
   \caption{\label{#2}#3}
   \end{figure}   
}

\renewcommand\@maketitle{%
\null
\vspace{-2cm}
\hbox{\raisebox{0.4em}{\vrule depth 0pt height 0.05cm width 17cm}}
\begin{center}
\Large \@title\\
\vspace{4mm}
\LARGE \@cours\\
\vspace{4mm}
\@author\\
\vspace{4mm}
\small\@email\\
\vspace{4mm}
\normalsize\@teacher
\vspace{-3mm}
\end{center}
\begin{figure}[h!]
	\centering
	\begin{subfigure}[b]{.49\linewidth}
		\centering
		\includegraphics[height=0.2\linewidth]{files/LCAV.png}
	\end{subfigure}
	\begin{subfigure}[b]{.49\linewidth}	
		\centering
		\includegraphics[height=0.2\linewidth]{files/EPFL.png}
	\end{subfigure}
\end{figure}
\begin{flushright}
	\today
\end{flushright}
\hbox{\raisebox{0.4em}{\vrule depth 0pt height 0.05cm width 17cm}}
}

%Foot and header

\RequirePackage{fancyhdr}
\pagestyle{fancy}
\fancyhead{} % clear all header fields
\fancyhead[EL,OR]{\slshape \rightmark} % add title left on even pages, right on odd pages
\fancyhead[ER,OL]{\@cours} %add page number right on even pages, left on odd pages
\fancyfoot[C]{\thepage}
