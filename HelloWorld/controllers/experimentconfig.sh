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
# 定义机器人数量
export NUMROBOTS=2

# 定义控制器脚本的路径
export CON1="${EXPERIMENTFOLDER}/controllers/main.py"  # 控制器脚本 main.py 的路径
# export CON3="${EXPERIMENTFOLDER}/controllers/main1.py"  # 控制器脚本 main.py 的路径

# 定义机器人通信范围
export RABRANGE="0.8"  # 机器人通信范围为 0.8 米

# 定义轮子噪声
export WHEELNOISE="0"  # 轮子噪声为 0，表示没有噪声

# 定义每秒步数
export TPS=30  # 每秒步（帧）数为 100

# 定义机器人密度
export DENSITY="1"  # 机器人密度为 1


# 定义竞技场的尺寸
export ARENADIM="2"  # 竞技场的总尺寸
export ARENADIMH="1"  # 竞技场的一半尺寸

# 定义实体的起始分布范围
export STARTDIM="0.8"  # 实体起始分布的维度

# NEW PARAM FOR WALLS
export REDUCEDL="1.85"  # 墙壁的缩减长度
export MOVEWALL="0.0"  # 墙壁移动的距离

# [GETH]
export BLOCKPERIOD=2  # 区块生成周期

# [SC]
export MAXWORKERS=15  # 最大工人数
export LIMITASSIGN=2  # 分配限制

export DEMAND_A=0  # 需求 A 的初始值
export DEMAND_B=1000  # 需求 B 的初始值
export REGENRATE=20  # 再生率
export FUELCOST=100  # 燃料成本
# 计算配额的临时变量
export QUOTA_temp=$(echo " scale=4 ; (75/$REGENRATE*$BLOCKPERIOD+0.05)/1" | bc)
# 计算最终配额
export QUOTA=$(echo "$QUOTA_temp*10/1" | bc)
export QUOTA=200  # 配额
export EPSILON=15  # 误差
export WINSIZE=5  # 窗口大小

# [OTHER]
export SEED=1500  # 随机种子
export TIMELIMIT=100  # 时间限制
export LENGTH=30
export SLEEPTIME=5  # 睡眠时间
export REPS=1
export NOTES="Variation of utility of the resource between 100 and 400"



