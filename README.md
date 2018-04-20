# b-skojarzenia b-adoratorów
Problem b-skojarzeń (b-matching) uogólnia problem skojarzeń dopuszczając skojarzenie każdego wierzchołka \(v\) w grafie \(G=(V,E)\) z co najwyżej \(b(v)\) wierzchołkami (czyli standardowy problem skojarzeń to problem b-skojarzeń z b(v)=1 dla każdego \(v\)). Zajmiemy się problemem b-skojarzeń w nieskierowanym grafie ważonym \(w(E)\). Celem jest znalezienie skojarzenia o maksymalnej sumarycznej wadze krawędzi. Algorytmy dokładne dla tego problemu istnieją, ale mają dużą złożoność lub są trudne w implementacji.  

Algorytm b-adoratorów (KHAN, Arif, et al, 2016) jest szybkim i ~prostym~ w implementacji rozwiązaniem problemu b-skojarzeń. Algorytm ten jest 2-aproksymacją: w najgorszym przypadku zwróci rozwiązanie co najwyżej dwa razy gorsze od optymalnego.
(https://www.cs.purdue.edu/homes/apothen/Papers/bMatching-SISC-2016.pdf)

Projekt przedstawia implementację współbieżnego algorytmu b-adoratorów z optymalizacjami opisanymi w powyższej pracy naukowej.  
Projekt powstał w ramach przedmiotu Programowanie Współbieżne.
