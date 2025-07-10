import random

from cwj_util import bin_packing

def main():
    capa = 100
    weights = [random.randint(10, 20) for _ in range(40)]
    log = True
    obj = bin_packing(weights, capa, log)
    
if __name__ == "__main__":
    main()