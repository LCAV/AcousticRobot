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
\usepackage[utf8]{inputenc}
\usepackage{lmodern}
\usepackage[T1]{fontenc}
\usepackage[british,UKenglish,USenglish,english,american]{babel}
\usepackage[framed]{mcode}
\usepackage{verbatim}  % Needed for the "comment" environment to make LaTeX comments
\usepackage[section]{placeins} % adds float barrier with each section
\usepackage{float}

%mathematics packages
\usepackage{array}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage[version=3]{mhchem}
\usepackage[retainorgcmds]{IEEEtrantools}

%Figures packages
\usepackage{graphicx}
\usepackage{wrapfig}
\usepackage{subcaption}
%\usepackage{subfigure}
%\usepackage[center]{subcaption}
%\usepackage[center]{subfigure}
\usepackage[font={small,it}]{caption}

%font-codes package
\usepackage{color}
\usepackage{comment}
\usepackage{hyperref}

% code listings
\usepackage{listings}
\usepackage{xcolor}
\lstset{
numbers=left, 
frame=lrtb,
numbersep=5pt,
frame=single,
breaklines=true,
postbreak=\raisebox{0ex}[0ex][0ex]{\ensuremath{\color{blue}\hookrightarrow\space}}
}
\renewcommand{\lstlistingname}{Code}

% Paper size and Layout

\usepackage[dvips, 
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
\usepackage{tabularx}
\usepackage{booktabs} % in order to use midrule, toprule etc.
\usepackage{multirow}
\usepackage{multicol}

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

\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhead{} % clear all header fields
\fancyhead[EL,OR]{\slshape \rightmark} % add title left on even pages, right on odd pages
\fancyhead[ER,OL]{\@cours} %add page number right on even pages, left on odd pages
\fancyfoot[C]{\thepage}
