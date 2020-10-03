#!/bin/bash

		#################################################
		#                                               #
		#     ARQUITECTURA DE COMPUTADORS AVANCADES     #
		#             		PRACTICA 2                  #
		#                                               #
		#          Estudiant: Jordi Bericat Ruz         #
		#                                               #
		#          Arxiu 1 de 3: submit_jobs.sh         #
		#                                               #
		#    SCRIPT PER A L'ENVIAMENT DE TASQUES AL     #
		#          GESTOR DE CUES DEL CLUSTER           #
		#                                               #
		#                   Versio 3.0                  #
		#                                               #
		#################################################


###############################################################################
#                                                                             #
#  DESCRIPCIO D'AQUEST SCRIPT:                                                #
#                                                                             #
#  Aquest script (que anomenarem "script principal") s'encarrega d'encuar     #
#  els diferents jobs al cluster SGE en funcio de diferents parametres (num.  #
#  de threads, num. de cores, path i executabe del workload o NPB benchark),  #
#  els quals es proporcionaran com a variables d'entorn a l'script de         #
#  configuracio del job corresponent (fitxer "SGE.conf").                     #
#                                                                             #
#  ALGORISME IMPLEMENTAT:                                                     #
#                                                                             #
#  A cada iteracio exterior del bucle niuat s'assignara una prova NPB         #
#  diferent (emmagatzemades a un vector) a la variable d'entorn $NPB_EXE.     #
#  Seguidament, per a cada iteracio del bucle interior, s'assignara el        #
#  nombre de threads amb el que es fara cada prova (3 per NPB) a la variable  #
#  d'entorn $NPB_CORES. Finalment s'enviara la tasca al gestor de cues del    #
#  cluster utilitant l'script de configuracio d'opcions per a                 #
#  "qsub [OPCIONS] SGE.conf".                                                 #
#                                                                             #
#  ENTRADA: llista d'executables NPB mijançant opció "-b" desde CLI           #
#                                                                             #
#  SORTIDA:                                                                   #
#                                                                             #
#  fitxers "job${NPB_JOB}.oXXXXXXX" -> Resultats del benchmark per a cada NPB #
#  i nivell de paralelitzacio                                                 #
#                                                                             #
#  fitxers "error.log" -> Log d'errors per a cada benchmark                   #
#                                                                             #
#  SINTAXI I EXEMPLE D'US: Veure ajuda amb ./submit_jobs.sh --help            #
#                                                                             #
###############################################################################


# INICI DE L'SCRIPT

##### Declaracio i inicialitzacio de variables
  
PIN_ROOT="/home/acaa06/apps/pin-2.12-58423-gcc.4.4.7-linux"
NPB_PATH="/share/apps/aca/benchmarks/NPB3.2/NPB3.2-OMP/bin"
NPB_JOB=0 #indica el num. x de y NPB's en total
NPB_CORES=
NPB_MAXTHREADS=
NPB_LIST=()
L1_SIZE=
L1_ASOC=
L1_ENTRY_SIZE=

##### Funcions auxiliars 

# Funcio "usage()" Imprimeix ajuda desde la CLI (modificadors -h, --help)

usage()
{
    echo "Usage: submit_jobs.sh [[[-c NUM_CORES ] [-t MAX_THREADS] [-b NPB_1.CLASS -b NPB_2.CLASS ... -b NPB_n.CLASS]] | [-h]]"
	echo;echo "Sends custom jobs to the SGE grid."
	echo;echo "Example: ./submit_jobs.sh -c 4 -t 4 -b is.S -b is.W"
	echo;echo "ALL options are mandatory."
	echo "-b NPB, --benchmark NPB 	NPB executable to submit to the grid's node" 
	echo "				  (you may use as much -b options as you like)." 
	echo "-c CORES, --cores CORES		number of cores availables for all benchmarks."
	echo "-h, --help			Print a help message and exit."
	echo "-t THREADS, --threads THREADS	maximum number of openMP threads (power of 2)." 
	echo "				  Since --threads is cumulative, then if you"
	echo "				  specify, let's say, \"-t 4\", you'll be sending"
	echo "				  log base 2 of 4 threads (that is 3 jobs, for"
	echo "				  1, 2 & 4 threads respectively)."
}

# Funcio "log2" (calcula nombre iteracions bucle interior en funcio del nombre
# maxim de threads indicat com argument d'aquest script a la CLI ($1)

function log2 {
    local x=0
    for (( y=$1-1 ; $y > 0; y >>= 1 )) ; do
        let x=$x+1
    done
    echo $x
}

##### Main

# Fem la gestio dels arguments com a opcions de l'script desde la CLI

while [ "$1" != "" ]; do
    case $1 in
        -c | --cores )        	shift
                                NPB_CORES=$1
                                ;;
        -t | --threads )    	shift
								NPB_MAXTHREADS=$1
                                ;;
        -b | --benchmark )    	shift
								NPB_LIST+=($1)
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

# Obtenim variables dcache d'entrada std

read -p "Especifica la mida de la cache L1 en kilobytes: " L1_SIZE
read -p "Especifica l'associativitat de la cache (enter potencia de 2): " L1_ASOC
read -p "Especifica la mida de cada entrada (linia) de cache L1 en bytes: " L1_ENTRY_SIZE

# Per a cada NPB benchmark emmagatzemat al vector $list_NPB...

for ((m=0 ; m<${#NPB_LIST[@]} ; m++))
do
	NPB_EXE=${NPB_LIST[$m]}

	# ... Efectuem el benchmark per a 1, 2 i 4 openMP threads

	for ((n=0 ; n<=$(log2 ${NPB_MAXTHREADS}) ; n++))
	do
		# Definim el nombre de threads (2^n) i el nom per aquest job

		NPB_THREADS=$[2**$n]
		NPB_JOB=$((NPB_JOB+1))

		# Enviem el job a la cua del SGE 

		qsub -N job${NPB_JOB} -pe openmp ${NPB_CORES} -v PIN_ROOT=${PIN_ROOT},NPB_EXE=${NPB_EXE},NPB_PATH=${NPB_PATH},NPB_THREADS=${NPB_THREADS},L1_SIZE=${L1_SIZE},L1_ASOC=${L1_ASOC},L1_ENTRY_SIZE=${L1_ENTRY_SIZE} SGE.conf
	done
done

# FINAL DEL SCRIPT

###############################################################################
#
# TO-DO SECTION (implementacions pendents per a la versio 4.0)
#
# TO-DO #1 
#
# Execució d'script de postproces un cop s'han executat tots els jobs anteriors
#
# (Afegir la següent crida a qsub al final de l'script):
#
# qsub -W depend=afterok:$JOBIDS postprocessing.sh
#
# Fonts -> https://stackoverflow.com/questions/31669161/automatic-qsub-job-completion-status-notification
#
# TO-DO #2  
#
# Mirar la manera de passar la opció -b desde la CLI en una sola opcio "-b",
# tal que:
#
# ./submit_jobs.sh -c 4 -t 8 -b=is.S,is.W,is.A,is.B
#
# TO-DO #3  
#
# Implementar control d'errors per a les opcions passades des de la CLI
#
###############################################################################