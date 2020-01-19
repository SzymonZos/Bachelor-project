import re
import os
from contextlib import ExitStack


def main():
    filepath = r'../thesis/chapters/'
    filenames = os.listdir(filepath)
    with ExitStack() as stack:
        files = [stack.enter_context(open(filepath + filename, 'r', encoding='UTF-8')) for filename in filenames]
        texfiles_copy = {filename: file.read() for file, filename in zip(files, filenames)}

    for filename, file_content in texfiles_copy.items():
        listings = []
        for match in re.finditer(r'\\begin{listing\}.+?\\end{listing\}', file_content, re.S | re.M):
            listings.append((match.start(), match.end()))
        print(listings)
        for match in re.finditer(r'((?<= \w) )|((?<= \w\w) )', file_content):
            for listing in listings:
                if listing[0] < match.start() < listing[1]:
                    break
            else:
                texfiles_copy[filename] = texfiles_copy[filename][:match.start()] + '~' +\
                                          texfiles_copy[filename][match.start()+1:]

    for filename in texfiles_copy:
        with open(filepath + filename, 'w', encoding='UTF-8') as file:
            file.write(texfiles_copy[filename])


if __name__ == '__main__':
    main()
