dataset='fr1_floor'
task='task2'
for i in $(seq 1 5)
do
    echo "Iteration $i"
    mkdir -p trajectories/trajectory_$dataset\_$task
    ./Apps/mono_tumrgbd /home/cesar/MRGCV/SLAM/datasets/$dataset > test.txt
    mv trajectory.txt trajectories/trajectory_$dataset\_$task/trajectory_$dataset\_$task\_$i.txt
done