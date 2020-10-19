def DPL_2_A():
    V, E = map(int, input().split())
    cost = [[float('inf')]*V for _ in range(V)]
    for _ in range(E):
        s, t, d = map(int, input().split())
        cost[s][t] = d

    dp = [[float('inf')]*V for _ in range(1 << V)]
    dp[1][0] = 0
    for S in range(1, 1 << V):
        for v in range(V):
            if not S & 1 << v:
                for u in range(V):
                    if S >> u & 1:
                        dp[S ^ 1 << v][v] = min(dp[S ^ 1 << v][v], dp[S][u] + cost[u][v])

    ans = min(dp[-1][v] + cost[v][0] for v in range(V))
    print(ans if ans < float('inf') else -1)

def sq_1_G():
    N, M = map(int, input().split())
    cost = [[float('inf')]*N for _ in range(N)]
    times = [[0]*N for _ in range(N)]
    for _ in range(M):
        s, t, d, time = map(int, input().split())
        s -= 1
        t -= 1
        cost[s][t] = d
        cost[t][s] = d
        times[s][t] = time
        times[t][s] = time

    dp = [[(float('inf'), 0)]*N for _ in range(1 << N)]
    dp[1][0] = (0, 1)
    for S in range(1 << N):
        for v in range(N):
            if not S >> v & 1:
                for u in range(N):
                    if S >> u & 1:
                        if dp[S][u][0] + cost[u][v] > times[u][v]:
                            continue
                        if dp[S ^ 1 << v][v][0] > dp[S][u][0] + cost[u][v]:
                            dp[S ^ 1 << v][v] = (dp[S][u][0] + cost[u][v], dp[S][u][1])
                        elif dp[S ^ 1 << v][v][0] == dp[S][u][0] + cost[u][v]:
                            dp[S ^ 1 << v][v] = (dp[S ^ 1 << v][v][0], dp[S ^ 1 << v][v][1] + dp[S][u][1])

    min_time, nums = float('inf'), 0
    for v in range(N):
        if dp[-1][v][0] + cost[v][0] > times[v][0]:
            continue
        if dp[-1][v][0] + cost[v][0] < min_time:
            min_time = dp[-1][v][0] + cost[v][0]
            nums = dp[-1][v][1]
        elif dp[-1][v][0] + cost[v][0] == min_time:
            nums += dp[-1][v][1]

    if min_time < float('inf'):
        print(min_time, nums)
    else:
        print("IMPOSSIBLE")

def JOI_13_yosen_D():
    MOD = 10007

    N = int(input())
    R = 'J' + input()

    memberToInt = {'J':1, 'O':2, 'I':4}

    dp = [[0]*(1 << 3) for _ in range(N + 1)]
    dp[0][1] = 1
    for i in range(1, N + 1):
        x = memberToInt[R[i]]
        for S in range(1 << 3):
            if not S & x:
                dp[i][S] = 0
                continue
            for T in range(1 << 3):
                if S & T:
                    dp[i][S] += dp[i - 1][T]
                    dp[i][S] %= MOD

    print(sum(dp[N]) % MOD)

if __name__ == "__main__":
    pass