# Get as input the dataset name
dataset_name=$1
echo "Dataset name: $dataset_name"
total_frames=0
std_dev=0
for i in $(seq 1 5)
do
    # Count the lines in the file
    num_lines=$(wc -l < trajectory_$dataset_name\_$i.txt)
    total_frames=$((total_frames + num_lines))
done
mean=$(echo "scale=2; $total_frames / 5" | bc)

for i in $(seq 1 5)
do
    # Count the lines in the file
    num_lines=$(wc -l < trajectory_$dataset_name\_$i.txt)
    std_dev=$(echo "$std_dev + ($num_lines - $mean) * ($num_lines - $mean)" | bc -l)
done
std_dev=$(echo "scale=2; $std_dev / 5" | bc -l)
std_dev=$(echo "scale=2; sqrt($std_dev)" | bc -l)
echo "Mean: $mean"
echo "Standard deviation: $std_dev"