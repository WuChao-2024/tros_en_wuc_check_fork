import os
import numpy as np
from openai import OpenAI

os.environ["OPENAI_API_KEY"] = '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'
# export OPENAI_API_KEY="[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]"


class Translate:
    def __init__(self):
        self.client = OpenAI()
        self.promot = "请把以下内容中出现中文的部分翻译成英文，保留原有的格式和内容："
        
        # 测试大模型状态
        self.completion = self.client.chat.completions.create(model="gpt-3.5-turbo",
    messages=[{"role": "system", "content": "你是优秀专业的翻译专家"},
        {"role": "user", "content": "你是谁？"}])
        print(self.completion.choices[0].message.content)
    
    def cn2en(self,english_content):
        try:
            input_contant = self.promot+english_content
            self.completion = self.client.chat.completions.create(model="gpt-3.5-turbo",
            messages=[{"role": "system", "content": "你是专业、优秀的翻译专家"},
            {"role": "user", "content": input_contant}])
            return self.completion.choices[0].message.content     

        except Exception as e:
            print(f"Error :{e}")
            return None 
        
        
    def execute_with_retry(self,final_data):
        max_attempts = 3
        current_attempt = 1

        while current_attempt <= max_attempts:
            result = self.cn2en(final_data)

            if result is not None:
                # 函数执行成功
                return result
            else:
                # 函数执行失败，尝试再次执行
                print(f"Retrying... Attempt {current_attempt}/{max_attempts}")
                current_attempt += 1

        print("Max attempts reached. Unable to execute the function.")
        return None
    
    def doc_split_and_save(self,input_file_path,save_file_path):
        with open(input_file_path, 'r', encoding='utf-8') as md_file:
            lines = md_file.readlines()

            num_lines = len(lines)
            batch_size = 50
            num_batches = (num_lines + batch_size - 1) // batch_size

            for i in range(num_batches):
                print("start load, now is : ",i,' all is : ',num_batches)
                start_idx = i * batch_size
                end_idx = (i + 1) * batch_size
                batch_lines = lines[start_idx:end_idx]
                #拆开临时文档并保存
                with open('catch.txt', 'w', encoding='utf-8') as file:
                    for line_element in batch_lines:
                        file.write(line_element)                 
                #从临时文件中读取文件写入到最终文件中
                with open(save_file_path, 'a', encoding='utf-8') as final_file:
                    with open('catch.txt', 'r', encoding='utf-8') as file:
                        final_data = file.read()   
                        # print(final_data)
                        last_write = self.execute_with_retry(final_data)
                        if last_write is not None:               
                            final_file.write(last_write)
                        else:
                            final_file.write(final_data)

        return 0        

    def read_and_save_md_files(self,root_folder, output_folder):
        try:
            max_files = []  # List to store the three largest files
            for root, dirs, files in os.walk(root_folder):
                print(f"Reading files in {root}")
                for file in files:
                    if file.endswith('.md') or file.endswith('.json'):
                    # if  file.endswith('.json'):

                        file_path = os.path.join(root, file)
                        # with open(file_path, 'r', encoding='utf-8') as md_file:
                        #     markdown_content = md_file.read()

                            # Determine relative path from the root_folder
                        relative_path = os.path.relpath(file_path, root_folder)
                        # Save content to the 'b' folder with the same relative pat
                        # name of file
                        save_path = os.path.join(output_folder, relative_path)
                        # name o dir 
                        save_folder = os.path.dirname(save_path)
                        os.makedirs(save_folder, exist_ok=True)
                        print(relative_path,save_path,save_folder)
                        
                        self.doc_split_and_save(file_path,save_path)
                        # with open(save_path, 'w', encoding='utf-8') as save_file:
                        #     save_file.write(markdown_content)

                        print(f"Content of {file_path} saved to {save_path}\n")

                            # Update max_files list with file size and path
                        file_size = os.path.getsize(file_path)
                        max_files.append((file_path, file_size))
                        os.remove(file_path)

            # Get the three largest files
            largest_files = sorted(max_files, key=lambda x: x[1], reverse=True)[:3]
            print("\nThree largest files:")
            for file_path, size in largest_files:
                print(f"{file_path} - Size: {size} bytes")

        except Exception as e:
            print(f"An error occurred: {e}")
            
if __name__=="__main__":
    translate = Translate()
    # translate.doc_split_and_save('RDK.md','en.md')
    translate.read_and_save_md_files('docs','en_docs')
