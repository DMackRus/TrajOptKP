
import sys
import cv2
import os

def create_video_from_frames(folder_path, output_path, fps=30):
    frame_files = os.listdir(folder_path)
    frame_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))

    frame = cv2.imread(os.path.join(folder_path, frame_files[0]))
    height, width, _ = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for filename in frame_files:
        image_path = os.path.join(folder_path, filename)
        frame = cv2.imread(image_path)
        video.write(frame)

    cv2.destroyAllWindows()
    video.release()

# Example usage


def main():
    print("hello world")
    
    folder_name = ''
    video_name = ''
    
    if(len(sys.argv) < 2):
        print("program needs arguments: {folder_name, video_name}")
    else:
        # argument 0 is the program name
        folder_name = sys.argv[1]
        video_name = sys.argv[2]
    
        folder_path = "../../media/videos/" + folder_name
        output_path = "../../media/videos/" + video_name + ".mp4"
        create_video_from_frames(folder_path, output_path)
    
    
if __name__ == "__main__":
    main()