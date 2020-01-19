import numpy as np
import re
import os
from contextlib import ExitStack



def power_iteration(A, num_simulations: int):
    # Ideally choose a random vector
    # To decrease the chance that our vector
    # Is orthogonal to the eigenvector
    b_k = np.random.rand(A.shape[1])
    print(b_k)

    for _ in range(num_simulations):
        # calculate the matrix-by-vector product Ab
        b_k1 = np.dot(A, b_k)

        # calculate the norm
        b_k1_norm = np.linalg.norm(b_k1)
        # print('my norm: ' + str(np.sqrt(np.dot(b_k1, b_k1))))

        # re normalize the vector
        b_k = b_k1 / b_k1_norm

    return np.dot(np.dot(A, b_k), b_k) / np.dot(b_k, b_k)


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
            print(texfiles_copy[filename])


if __name__ == '__main__':
    main()
