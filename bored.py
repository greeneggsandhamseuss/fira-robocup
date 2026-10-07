name = input("Hi, what's your name? ").strip().capitalize()
print(f"Hey there, {name}! I'm Mister Robot and I'd like to get to know you!")

age = input(f'How old are you, {name}? ').strip()
if age == 21:
    print(f"dang you unc, {name}! get out of here!")
