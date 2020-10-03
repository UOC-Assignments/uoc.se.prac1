#!/bin/bash 

		#################################################
		#                                               #
		#     ARQUITECTURA DE COMPUTADORS AVANCADES     #
		#             PRACTICA 1 - ENTREGA 2            #
		#                                               #
		#          Estudiant: Jordi Bericat Ruz         #
		#                                               #
		#        Arxiu 3 de 3: postprocessing.sh        #
		#                                               #
		#    SCRIPT DE CREACIO DE FITXER DE SORTIDA     #
		#                 "output.log"                  #
		#                                               #
		#                  Versio 2.0                   #
		#                                               #
		#################################################

###############################################################################
#                                                                             #
#  DESCRIPCIO D'AQUEST SCRIPT:                                                #
#                                                                             #
#  Aquest script s'executara de manera manual un cop s'hagin processat tots   #
#  els jobs enviats al SGE per tal de generar el fitxer definitiu de sortida  #
#  output.log. (TO-DO: A la versió 3.0 es tractara d'implementar la execucio  #
#  automatica mitjançant l'us de la opció "-W" de la comanda "qsub" des de    #
#  l'script principal "submit_jobs.sh"                                        #
#                                                                             #
#  ENTRADA: fitxers job*.o*                                                   #
#                                                                             #
#  SORTIDA: fitxer "output.log" amb el resultat de tots els benchmarks        #
#           agrupats per NPB i ordenats de menys a mes threads                #
#                                                                             #
###############################################################################

# INICI DE L'SCRIPT 

cat $(find ./ -name "job*.o*" | sort -V) > output.log
rm job*

# FINAL DE L'SCRIPT 