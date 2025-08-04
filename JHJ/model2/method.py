from __future__ import annotations
from typing import TYPE_CHECKING
from local_search import RelocateMove, ExchangeMove, CVRPBSolutionNodeIterator
import time

if TYPE_CHECKING:
    from Solution import CVRPBSolution

class FirstRelocate:
    """
    RelocateMove를 평가하여 이득(gain)을 주는 첫 번째 이동을 찾아 즉시 적용합니다.
    """
    def minimize(self, solution: CVRPBSolution):
        from_iter = CVRPBSolutionNodeIterator(solution)
        while from_iter.has_next():
            from_iter.next()
            if from_iter.last_route() is None:
                continue
            to_iter = CVRPBSolutionNodeIterator(solution)
            while to_iter.has_next():
                to_iter.next()
                if to_iter.last_route() is None:
                    continue
                move = RelocateMove(solution, from_iter, to_iter)
                if move.is_legal():
                    gain = move.gain()
                    if gain > 1e-9:
                        move.apply()
                        solution.update_cost_with_gain(gain) # 비용 직접 업데이트
                        return True # 개선이 있었음을 알림
        return False



class FirstExchange:
    """
    ExchangeMove를 평가하여 이득(gain)을 주는 첫 번째 교환을 찾아 즉시 적용합니다.
    """
    def minimize(self, solution: CVRPBSolution):
        iter1 = CVRPBSolutionNodeIterator(solution)
        while iter1.has_next():
            iter1.next()
            if iter1.last_route() is None:
                continue

            iter2 = CVRPBSolutionNodeIterator(solution)
            for _ in range(iter1.last_node_index()):
                if iter2.has_next():
                     iter2.next()
                else: break
            
            while iter2.has_next():
                iter2.next()
                if iter2.last_route() is None:
                    continue
                    
                if iter1.last_route() == iter2.last_route() and iter1.last_node_index() == iter2.last_node_index():
                    continue

                move = ExchangeMove(solution, iter1, iter2)
                if move.is_legal():
                    gain = move.gain()
                    if gain > 1e-9:
                        move.apply()
                        solution.update_cost_with_gain(gain) # 비용 직접 업데이트
                        return True # 개선이 있었음을 알림
        return False


class FirstImprovementStrategy:
    """
    FirstRelocate와 FirstExchange를 번갈아 적용하여 해를 개선합니다.
    개선이 발견되면 즉시 처음부터 다시 탐색을 시작합니다.
    """
    def __init__(self):
        self.first_relocate = FirstRelocate()
        self.first_exchange = FirstExchange()

    def minimize(self, solution: CVRPBSolution, start_time: float, time_limit: float):
        # 최초 비용 계산
        solution.get_total_cost()

        while True:
            if time.time() - start_time > time_limit:
                print("Time limit reached. Terminating search.")
                break

            improved_by_relocate = self.first_relocate.minimize(solution)
            if improved_by_relocate:
                continue

            improved_by_exchange = self.first_exchange.minimize(solution)
            if improved_by_exchange:
                continue
            
            break
