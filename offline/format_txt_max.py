import os

folder_path = "archive"  # Change this to your actual folder path

for filename in os.listdir(folder_path):
    if filename.endswith(".txt"):
        file_path = os.path.join(folder_path, filename)
        
        with open(file_path, "r") as file:
            lines = file.readlines()
        
        with open(file_path, "w") as file:
            for line in lines:
                cleaned_line = line.replace(",", " ")  # Remove all commas
                file.write(cleaned_line)
