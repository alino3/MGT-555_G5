import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import numpy_financial as npf

# ==========================================
# 1. CONFIGURATION & ASSUMPTIONS
# ==========================================
YEARS = 5
DISCOUNT_RATE = 0.10  # 10% Risk rate
TAX_RATE = 0.20       # Corporate Tax
INITIAL_INVESTMENT = 100000

# --- Pricing (Consistent across all scenarios) ---
PRICE_HARDWARE_KIT = 2500      # Cameras, Sensors, Mounts
PRICE_INTEGRATION_FEE = 5000   # Setup & Licensing
PRICE_SUBSCRIPTION = 4000      # Annual AI License (Recurring)
COGS_HARDWARE = 700            # Variable cost per unit

# --- Fixed Costs (Annual OPEX) ---
# Breakdown for visualization
FIXED_COSTS_BREAKDOWN = {
    'Development (AI/Eng)': 180000,
    'Admin': 160000,
    'HQ': 25000,
    'Marketing': 15000,
    'Others': 10000,
    'Super-computer': 10000
}
TOTAL_FIXED_COSTS_NORMAL = sum(FIXED_COSTS_BREAKDOWN.values()) # $430,000

# ==========================================
# 2. CALCULATION FUNCTIONS
# ==========================================

def calculate_scenario(name, volume_schedule, fixed_costs_schedule):
    """Calculates 5-year financials for a given scenario."""
    years_arr = np.arange(1, YEARS + 1)
    new_units = np.array(volume_schedule)
    cum_units = np.cumsum(new_units)
    
    # Revenue Streams
    rev_hw = new_units * PRICE_HARDWARE_KIT
    rev_int = new_units * PRICE_INTEGRATION_FEE
    rev_sub = cum_units * PRICE_SUBSCRIPTION
    total_rev = rev_hw + rev_int + rev_sub
    
    # Costs
    cogs = new_units * COGS_HARDWARE
    gross_profit = total_rev - cogs
    opex = np.array(fixed_costs_schedule)
    
    # Profits
    ebitda = gross_profit - opex
    taxes = np.maximum(0, ebitda * TAX_RATE)
    net_income = ebitda - taxes
    
    # Cash Flow
    cash_flows = np.concatenate(([-INITIAL_INVESTMENT], net_income))
    cum_cash_flow = np.cumsum(cash_flows)
    
    # Metrics
    npv = npf.npv(DISCOUNT_RATE, cash_flows)
    try:
        irr = npf.irr(cash_flows)
    except:
        irr = np.nan
        
    return {
        'name': name,
        'years': years_arr,
        'new_clients': new_units,
        'rev_hw': rev_hw,
        'rev_int': rev_int,
        'rev_sub': rev_sub,
        'total_rev': total_rev,
        'ebitda': ebitda,
        'net_income': net_income,
        'cash_flows': cash_flows,
        'cum_cash_flow': cum_cash_flow,
        'npv': npv,
        'irr': irr
    }

def calculate_long_term_scenario(base_vol_schedule):
    """Extends the Base Case to 15 years with churn and maintenance growth."""
    long_term_years = 15
    churn_rate = 0.05
    
    # Extend Volume Schedule: Ramp up (Y1-5) + Maintenance (Y6-15)
    vol_ramp = np.array(base_vol_schedule)
    vol_maint = np.full(long_term_years - 5, 20) # 20 new units/year after Y5
    new_units = np.concatenate((vol_ramp, vol_maint))
    
    cash_flows = [-INITIAL_INVESTMENT]
    total_clients = 0
    
    for t in range(long_term_years):
        current_year = t + 1
        
        # Client Logic: Add new, subtract churn (after Y5)
        if current_year <= 5:
            total_clients += new_units[t]
        else:
            total_clients = total_clients * (1 - churn_rate) + new_units[t]
            
        # Financials
        rev_upfront = new_units[t] * (PRICE_HARDWARE_KIT + PRICE_INTEGRATION_FEE)
        rev_sub = total_clients * PRICE_SUBSCRIPTION
        total_rev = rev_upfront + rev_sub
        
        cogs = new_units[t] * COGS_HARDWARE
        
        # Fixed Costs: Assume Optimized Low Cost in Y1-2, Normal afterwards
        fc = 330000 if current_year <= 2 else TOTAL_FIXED_COSTS_NORMAL
             
        ebitda = (total_rev - cogs) - fc
        taxes = np.maximum(0, ebitda * TAX_RATE)
        net_income = ebitda - taxes
        
        cash_flows.append(net_income)
        
    return np.array(cash_flows)

# ==========================================
# 3. PLOTTING FUNCTIONS
# ==========================================

def plot_cashflow_comparison(scenarios):
    plt.figure(figsize=(10, 6))
    years_plot = np.arange(0, YEARS + 1)
    
    for s in scenarios:
        print(f"{s['name']} Case - NPV: ${s['npv']:,.2f}, IRR: {s['irr']:.2%}")
        print(f" Cum CF: {s['cum_cash_flow']}")
        plt.plot(years_plot, s['cum_cash_flow'], marker='o', linewidth=2, 
                 label=f"{s['name']} (NPV: ${s['npv']/1000:.0f}k)")
        
    plt.axhline(0, color='black', linewidth=1)
    plt.title('5-Year Cumulative Cash Flow Comparison')
    plt.ylabel('Bank Balance ($)')
    plt.xlabel('Year')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig('final_cashflow_comparison.png')
    print("Saved: final_cashflow_comparison.png")

def plot_revenue_breakdown(scenario):
    years = scenario['years']
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Stacked Bar Chart
    p1 = ax.bar(years, scenario['rev_hw'], label='Hardware Kit ($2.5k)', color='#1f77b4', alpha=0.85)
    p2 = ax.bar(years, scenario['rev_int'], bottom=scenario['rev_hw'], label='Integration Fee ($5k)', color='#ff7f0e', alpha=0.85)
    p3 = ax.bar(years, scenario['rev_sub'], bottom=scenario['rev_hw'] + scenario['rev_int'], label='Subscription ($4k/yr)', color='#2ca02c', alpha=0.85)
    
    ax.set_ylabel('Revenue ($)')
    ax.set_xlabel('Year')
    ax.set_title(f"Revenue Sources Breakdown ({scenario['name']} Case)")
    ax.legend()
    ax.grid(axis='y', linestyle='--', alpha=0.3)
    
    # Add Total Labels
    total = scenario['total_rev']
    for i in range(len(years)):
        ax.text(years[i], total[i] + 15000, f"${total[i]/1000:.0f}k", ha='center', fontweight='bold')
        
    plt.tight_layout()
    plt.savefig('final_revenue_breakdown.png')
    print("Saved: final_revenue_breakdown.png")

def plot_cost_structure_year5(scenario):
    # Calculate Variable Cost for Year 5
    units_y5 = scenario['new_clients'][-1]
    var_cost = units_y5 * COGS_HARDWARE
    
    # Combine with Fixed Costs
    costs = FIXED_COSTS_BREAKDOWN.copy()
    costs['Variable (Hardware)'] = var_cost
    costs = dict(sorted(costs.items(), key=lambda item: item[1], reverse=True))
    
    plt.figure(figsize=(8, 8))
    plt.pie(costs.values(), labels=costs.keys(), autopct='%1.1f%%', startangle=140, colors=plt.cm.Paired.colors)
    plt.title('Cost Structure Breakdown (Year 5)')
    plt.tight_layout()
    plt.savefig('final_cost_structure.png')
    print("Saved: final_cost_structure.png")

def plot_long_term(cash_flows):
    years_plot = np.arange(0, len(cash_flows))
    cum_cash_flow = np.cumsum(cash_flows)
    
    # Simulate Uncertainty (Growing standard deviation)
    sigma = np.linspace(0, 300000, len(cash_flows)-1) 
    annual_ncf = cash_flows[1:]
    
    upper_cum = np.cumsum(np.concatenate(([cash_flows[0]], annual_ncf + sigma)))
    lower_cum = np.cumsum(np.concatenate(([cash_flows[0]], annual_ncf - sigma)))
    
    plt.figure(figsize=(12, 6))
    plt.plot(years_plot, cum_cash_flow, color='#1f77b4', linewidth=3, label='Base Forecast')
    plt.fill_between(years_plot, lower_cum, upper_cum, color='#94a6d4', alpha=0.3, label='Uncertainty Range')
    
    plt.axhline(0, color='black', linewidth=1)
    plt.title('Long-Term Cash Flow Projection (15 Years)')
    plt.ylabel('Cumulative Cash ($)')
    plt.xlabel('Year')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig('final_long_term.png')
    print("Saved: final_long_term.png")

def plot_rev_vs_ebitda(scenario):
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    ax1.set_xlabel('Year')
    ax1.set_ylabel('Revenue ($)', color='#1f77b4')
    ax1.bar(scenario['years'], scenario['total_rev'], color='#1f77b4', alpha=0.6, label='Revenue')
    
    ax2 = ax1.twinx()
    ax2.set_ylabel('EBITDA ($)', color='#2ca02c')
    ax2.plot(scenario['years'], scenario['ebitda'], color='#2ca02c', marker='o', linewidth=3, label='EBITDA')
    ax2.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    
    plt.title(f"{scenario['name']} Case: Revenue vs Profitability")
    fig.tight_layout()
    plt.savefig('final_rev_vs_ebitda.png')
    print("Saved: final_rev_vs_ebitda.png")

# ==========================================
# 4. MAIN EXECUTION
# ==========================================

# Define Scenarios
# Base (Optimized): 155 units, Reduced fixed costs ($330k) in Y1/Y2
vol_base = [15, 25, 35, 45, 35]
fc_normal = [430000] * 5
scen_base = calculate_scenario("Base", vol_base, fc_normal)

# Best: 200 units, Normal fixed costs ($430k)
vol_best = [30, 40, 50, 40, 40]
scen_best = calculate_scenario("Best", vol_best, fc_normal)

# Worst: 80 units, Normal fixed costs
vol_worst = [10, 20, 25, 30, 30]
scen_worst = calculate_scenario("Worst", vol_worst, fc_normal)

# Long Term Calculation
lt_cash_flows = calculate_long_term_scenario(vol_base)

# Generate Plots
print("--- Generating Plots ---")
plot_cashflow_comparison([scen_base, scen_best, scen_worst])
plot_revenue_breakdown(scen_base)
plot_cost_structure_year5(scen_base)
plot_long_term(lt_cash_flows)
plot_rev_vs_ebitda(scen_base)

# Print Summary
print("\n--- Final Financial Summary (Base Case) ---")
print(f"Total Customers (5Y): {sum(scen_base['new_clients'])}")
print(f"NPV: ${scen_base['npv']:,.2f}")
print(f"IRR: {scen_base['irr']:.2%}")