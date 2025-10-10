if [ !-d "output_video" ]; then
  printf "Creating output_video directory\n"
  mkdir output_video
fi

rm output_video/temp_image*
printf "Creating frames:"
printf "\t- Start frame: $1\n"
printf "\t- End frame: $2\n"
printf "\t- Video speed (60=realtime): $3\n"
python3 make_frames.py $1 $2 $3
printf "Frames created, making video from frames\n"
rm output_video/$4.mp4
ffmpeg -framerate 30 -pattern_type glob -i 'output_video/temp_image_*.png' -c:v libx264 output_video/$4.mp4
printf "Video created\n"