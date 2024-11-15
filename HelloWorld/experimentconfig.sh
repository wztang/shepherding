# [PATHS]
export HOMEFOLDER="$HOME/"
export MAINFOLDER="$HOMEFOLDER/Desktop/ARGoS/shepherding"
export DOCKERFOLDER="$MAINFOLDER/argos-blockchain-sm"
export ARGOSFOLDER="$MAINFOLDER/argos-python"
export EXPERIMENTFOLDER="$MAINFOLDER/HelloWorld"
export BLOCKCHAINPATH="$HOMEFOLDER/eth_data_para/data"
# [[ ":$PATH:" != *":$MAINFOLDER/scripts:"* ]] && export PATH=$PATH:$MAINFOLDER/scripts

# [FILES]
export ARGOSNAME="market-foraging"
export GENESISNAME="genesis_poa"
export CONTRACTNAME="MarketForaging"
export SCNAME="hello_neighbor"

export GENESISFILE="${DOCKERFOLDER}/geth/files/$GENESISNAME.json"
export CONTRACTADDRESS="${EXPERIMENTFOLDER}/scs/contractAddress.txt"
export CONTRACTABI="${EXPERIMENTFOLDER}/scs/build/$CONTRACTNAME.abi"
export CONTRACTBIN="${EXPERIMENTFOLDER}/scs/build/$CONTRACTNAME.bin-runtime"
export SCFILE="${EXPERIMENTFOLDER}/scs/${SCNAME}.sol" 
export SCTEMPLATE="${EXPERIMENTFOLDER}/scs/${SCNAME}.x.sol" 
export ARGOSFILE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.argos"
export ARGOSTEMPLATE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.x.argos"

# [DOCKER]
export SWARMNAME=ethereum
export CONTAINERBASE=${SWARMNAME}_eth

# [ARGOS]
export NUMROBOTS=
export CON1="${EXPERIMENTFOLDER}/controllers/main.py"


export RABRANGE="0.5"
export WHEELNOISE="0"
export TPS=15
export DENSITY="1"


export ARENADIM="2"
export ARENADIMH="1"
export STARTDIM="0.75"

#NEW PARAM FOR WALLS
export REDUCEDL="1.85"
export MOVEWALL="0.1"

# [GETH]
export BLOCKPERIOD=2

export NUMBYZANTINE=0
export BYZANTINESTYLE=1

# [SC]
export MAXWORKERS=15
export LIMITASSIGN=2

export DEMAND_A=0
export DEMAND_B=800
export REGENRATE=20
export FUELCOST=100
export QUOTA_temp=$(echo " scale=4 ; (75/$REGENRATE*$BLOCKPERIOD+0.05)/1" | bc)
export QUOTA=$(echo "$QUOTA_temp*10/1" | bc)
export QUOTA=200
export EPSILON=15
export WINSIZE=5

# [OTHER]
export SEED=1300
export TIMELIMIT=100
export LENGTH=5000
export SLEEPTIME=5
export REPS=5
export NOTES="Variation of utility of the resource between 100 and 400"




