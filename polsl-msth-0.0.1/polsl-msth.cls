%%
%% This is file `polsl-msth.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% polsl-msth.dtx  (with options: `class')
%% ----------------------------------------------------------------
%% polsl-msth --- Silesian University of Technology, Master of Science thesis
%% Author:  Krzysztof Mazur
%% E-mail:  krzysiek@podlesie.net
%% License: Released under the LaTeX Project Public License v1.3c or later
%% See:     http://www.latex-project.org/lppl.txt
%% ----------------------------------------------------------------
%% 
\NeedsTeXFormat{LaTeX2e}[1999/12/01]
\ProvidesClass{polsl-msth}
    [2018/01/12 v0.0.1-statement Silesian University of Technology, Ms thesis class]
\LoadClass[a4paper,twoside,openright,12pt]{book}
\linespread{1.3}
\RequirePackage{babel}
\setlength{\textwidth}{\paperwidth}
\setlength{\textheight}{\paperheight}
\setlength{\hoffset}{-0.04cm}
\setlength{\voffset}{-0.04cm}
\addtolength{\textwidth}{-5cm}
\addtolength{\textheight}{-5cm}
\setlength{\topmargin}{0cm}
\setlength{\headheight}{0cm}
\setlength{\headsep}{0cm}
\setlength{\marginparsep}{0cm}
\setlength{\marginparwidth}{0cm}
\setlength{\footskip}{0cm}
\setlength{\oddsidemargin}{0.5cm}
\setlength{\evensidemargin}{0cm}
\addtolength{\textwidth}{-0.5cm}
\setlength{\headheight}{12pt}
\setlength{\headsep}{19pt}
\addtolength{\textheight}{-\the\headheight}
\addtolength{\textheight}{-\the\headsep}
\def\authordesc{Author}
\def\supervisordesc{Supervisor}
\def\consultantdesc{Consultant}
\def\msthesisname{Master Thesis}
\def\engthesisname{Final Project}
\def\universitydefault{Silesian University of Technology}
\def\facultydefault{Faculty of Automatic Control, Electronics and Computer Science}
\def\@date{\ifcase\month\or
January\or February\or March\or April\or May\or June\or
July\or August\or September\or October\or November\or December\fi
\number\year}
\DeclareOption{polish}{%
\frenchspacing%
\def\authordesc{Autor}
\def\supervisordesc{Kieruj\k{a}cy prac\k{a}}%
\def\consultantdesc{Konsultant}
\def\msthesisname{Praca dyplomowa magisterska}
\def\engthesisname{Praca dyplomowa in\.zynierska}
\def\universitydefault{Politechnika \'Sl\k{a}ska}
\def\facultydefault{Wydzia\l\ Automatyki, Elektroniki i~Informatyki}
\def\@date{%
\ifcase\month\or
Stycze\'n\or Luty\or Marzec\or Kwiecie\'n\or Maj\or Czerwiec\or Lipiec\or
Sierpie\'n\or Wrzesie\'n\or Pa\'zdziernik\or Listopad\or Grudzie\'n\fi
\space\number\year%
}
}
\def\thesisname{\msthesisname}
\DeclareOption{engineer}{
\def\thesisname{\engthesisname}
}
\DeclareOption{master}{
\def\thesisname{\msthesisname}
}
\ProcessOptions*
\RequirePackage[T1]{fontenc}
\RequirePackage{lmodern}
\RequirePackage{graphicx}
\def\university#1{\gdef\@university{#1}}
\def\@university{\universitydefault}
\def\faculty#1{\gdef\@faculty{#1}}
\def\@faculty{\facultydefault}


\def\supervisor#1{\gdef\@supervisor{#1}}
\def\@supervisor{\@latex@warning@no@line{No \noexpand\supervisor given}}

\def\consultant#1{\gdef\@consultant{#1}}

\renewcommand\maketitle{
  \begingroup
    \renewcommand\thefootnote{\@fnsymbol\c@footnote}%
    \def\@makefnmark{\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
    \long\def\@makefntext##1{\parindent 1em\noindent
            \hb@xt@1.8em{%
                \hss\@textsuperscript{\normalfont\@thefnmark}}##1}%
      \newpage
      \global\@topnum\z@   % Prevents figures from going at top of page.
      \@maketitle
     \thispagestyle{empty}\@thanks
  \endgroup
  \setcounter{footnote}{0}%
  \global\let\thanks\relax
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@supervisor\@empty
  \global\let\@faculty\@empty
  \global\let\@university\@empty
  \global\let\title\relax
  \global\let\author\relax
  \global\let\supervisor\relax
  \global\let\faculty\relax
  \global\let\university\relax
  \global\let\date\relax
  \global\let\and\relax
}
\newcommand{\@bigfontsc}{\fontfamily{phv}\fontsize{18}{18}\bfseries\scshape\selectfont}
\newcommand{\@bigfont}{\fontfamily{phv}\fontsize{18}{18}\selectfont}
\newcommand{\@helfourty}{\fontfamily{phv}\fontsize{14}{14}\selectfont}
\def\@maketitle{%
\newpage%
\let\footnote\thanks%
\vbox to 22.910474truecm{% 24 cm minus header
\vbox to 17.910474truecm{% 19 cm minus header
\vbox to 32mm{
\vfill%
\centerline{\includegraphics[width=29mm]{pi}}
}
\vspace{1cm}
\begin{center}
\@bigfontsc \@university
\end{center}
\begin{center}
\@bigfontsc \@faculty
\end{center}
\vfil
\begin{center}
\@bigfont \thesisname
\end{center}
\vfil
{
\setlength{\baselineskip}{1.5\baselineskip}
\begin{center}
{\@helfourty \@title\par}
\end{center}
}
\vfil
}%
\noindent\vbox{%
\hbox{{\@helfourty\authordesc:} {\@helfourty \@author}}
\vspace{12pt}%
\hbox{{\@helfourty\supervisordesc:} {\@helfourty \@supervisor}}
\ifx\@consultant\undefined\else{
\vspace{12pt}%
\hbox{{\@helfourty\consultantdesc:} {\@helfourty \@consultant}}
}
\fi
}
\vfill%
}
\centerline{\@helfourty Gliwice, \@date}
}
\def\@makestatementdots#1{\hbox to #1{\cleaders\hbox to 0.8ex{$\hss.\hss$}\hfill}}
\def\makestatement{%
\cleardoublepage%
\let\footnote\thanks%
\vbox to 1.8cm{}
\centerline{\large\,\bf\,O\,\'S\,W\,I\,A\,D\,C\,Z\,E\,N\,I\,E}
\vspace{1.55cm}
\noindent Wyra\.zam zgod\k{e}/nie wyra\.zam* zgody na
udost\k{e}pnienie mojej pracy dyplomowej/rozprawy doktorskiej*

\vspace{2.3cm}
\noindent \@makestatementdots{4.5cm}, dnia \@makestatementdots{4.5cm}

\vspace{2.9cm}
\hfill \@makestatementdots{7.2cm}

\hfill \hbox to 7.2cm{\hfill\scriptsize \it (podpis)\hfill}

\vspace{1.3cm}
\hfill \@makestatementdots{7.2cm}

\hfill \hbox to 7.2cm{\hfill\scriptsize \it (po\'swiadczenie wiarygodno\'sci
podpisu przez Dziekanat)\hfill}\\

\vspace{1.5cm}
\noindent * w³a¶ciwe podkre¶liæ
\vfill%
}
\renewcommand\chapter{\if@openright\cleardoublepage\else\clearpage\fi
                    \thispagestyle{empty}%
                    \global\@topnum\z@
                    \@afterindentfalse
                    \secdef\@chapter\@schapter}
%% 
%% Copyright (C) 2017 by Silesian University of Technology.
%% Copyright (C) 2010, 2014 by Krzysztof Mazur <krzysiek@podlesie.net>
%% Fragments from book class by The LaTeX3 Project.
%% 
%% Author: Krzysztof Mazur <krzysiek@podlesie.net>
%% 
%% This work may be distributed and/or modified under the
%% conditions of the LaTeX Project Public License (LPPL), either
%% version 1.3c of this license or (at your option) any later
%% version.  The latest version of this license is in the file:
%% 
%% http://www.latex-project.org/lppl.txt
%% 
%% This work is "maintained" (as per LPPL maintenance status) by
%% Krzysztof Mazur.
%% 
%% This work consists of the file polsl-msth.dtx and a Makefile.
%% Running make generates the derived files README, polsl-msth.pdf and polsl-msth.cls.
%% Running make inst installs the files in the user's TeX tree.
%% Running make install installs the files in the local TeX tree.
%% 
%%
%% End of file `polsl-msth.cls'.
